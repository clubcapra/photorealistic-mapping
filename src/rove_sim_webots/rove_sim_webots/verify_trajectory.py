"""Run an auto-trajectory headless and cross-verify against obstacles.

For each (world, trajectory) pair:
  1. Spawn a small recorder that subscribes to /ground_truth/odom.
  2. Launch the sim+driver via scripted_runner --mode live --headless.
  3. After the trajectory completes, parse the recorded poses.
  4. Cross-verify: check every actual pose against world obstacles, compute
     deviation from the planned path, and report any collisions.

Output: per-world JSON summary at <out_dir>/<world>_verify.json.
"""
from __future__ import annotations

import json
import math
import os
import signal
import subprocess
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path

from rove_sim_webots.auto_trajectory import (
    parse_obstacles, parse_rove_pose, parse_arena_bounds, simulate_path,
    path_clearance, ROBOT_RADIUS_M, SAFETY_MARGIN_M,
)


WORLDS = [
    'indoor_office', 'indoor_warehouse', 'indoor_structured',
    'outdoor_urban', 'outdoor_terrain', 'outdoor_rocky', 'mixed',
]


@dataclass
class VerifyResult:
    world: str
    trajectory: str
    planned_length_m: float
    actual_length_m: float
    n_actual_poses: int
    min_actual_clearance_m: float
    max_deviation_m: float
    collisions: int  # poses where actual clearance < ROBOT_RADIUS
    status: str       # "pass" | "soft_fail" | "collision"


def _record_ground_truth(out_path: Path, domain_id: int = 125) -> subprocess.Popen:
    """Spawn a Python subscriber that writes /ground_truth/odom poses to JSON."""
    script = f"""
import json, sys, time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class Recorder(Node):
    def __init__(self):
        super().__init__('gt_recorder')
        self.poses = []
        self.create_subscription(Odometry, '/ground_truth/odom', self.cb, 10)
    def cb(self, msg):
        p = msg.pose.pose.position
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.poses.append({{'t': t, 'x': p.x, 'y': p.y, 'z': p.z}})

import os
os.environ['ROS_DOMAIN_ID'] = '{domain_id}'
rclpy.init()
node = Recorder()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    with open('{out_path}', 'w') as f:
        json.dump(node.poses, f)
    node.destroy_node()
    rclpy.shutdown()
"""
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = str(domain_id)
    env['PYTHONUNBUFFERED'] = '1'
    return subprocess.Popen(
        ['python3', '-c', script],
        env=env,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid,
    )


def _stop_recorder(proc: subprocess.Popen, timeout: float = 5.0):
    """SIGINT the recorder process group so it flushes its JSON cleanly."""
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        proc.wait()


def _cleanup_webots():
    subprocess.run(['/tmp/clean_webots.sh'], check=False,
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def _launch_headless_sim(world: str, domain_id: int, sim_log_path: Path,
                            mode: str = 'realtime',
                            webots_port: int = 1234,
                            ) -> subprocess.Popen:
    """Launch ONLY Webots sim (no rtabmap, no trajectory driver) headless.
    `mode` is passed through to sim.launch.py — 'realtime' or 'fast'.
    `webots_port` lets multiple Webots instances run in parallel."""
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = str(domain_id)
    env['WEBOTS_GUI'] = 'false'
    env['WEBOTS_PORT'] = str(webots_port)
    env.pop('DISPLAY', None)
    env.pop('WAYLAND_DISPLAY', None)
    env['QT_QPA_PLATFORM'] = 'xcb'
    env['PYTHONUNBUFFERED'] = '1'
    cmd = ['xvfb-run', '-a', '-s', '-screen 0 800x600x24',
           'ros2', 'launch', 'rove_sim_webots', 'sim.launch.py',
           f'world:={world}.wbt', f'mode:={mode}']
    return subprocess.Popen(
        cmd, env=env,
        stdout=open(sim_log_path, 'w'), stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )


def _run_waypoint_driver(traj_yaml: Path, domain_id: int,
                           log_path: Path, max_runtime_s: float
                           ) -> subprocess.Popen:
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = str(domain_id)
    env['PYTHONUNBUFFERED'] = '1'
    return subprocess.Popen([
        'python3', '-u', '-m', 'rove_sim_webots.waypoint_driver',
        str(traj_yaml),
        '--max-runtime-s', str(max_runtime_s),
    ], env=env,
        stdout=open(log_path, 'w'), stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )


def _stop_proc(proc: subprocess.Popen, timeout: float = 5.0):
    if proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        proc.wait()


def verify_world(world: str, out_dir: Path, domain_id: int = 125,
                  sim_mode: str = 'realtime',
                  webots_port: int = 1234,
                  skip_cleanup: bool = False) -> VerifyResult:
    """Run one world headless + record + verify. Returns a VerifyResult."""
    worlds_dir = Path(__file__).resolve().parent.parent / 'worlds'
    wbt_path = worlds_dir / f'{world}.wbt'
    obstacles = parse_obstacles(wbt_path)
    arena = parse_arena_bounds(wbt_path) or (-20.0, 20.0, -20.0, 20.0)
    start = parse_rove_pose(wbt_path)

    traj_name = f'auto_{world}'
    traj_yaml_path = (Path(__file__).resolve().parent.parent
                       / 'config' / 'trajectories' / f'{traj_name}.yaml')
    import yaml
    traj = yaml.safe_load(traj_yaml_path.read_text())

    # Build "planned path" for deviation comparison.
    if traj.get('type') == 'waypoints':
        planned_path = [(start[0], start[1])]
        planned_path.extend((float(p[0]), float(p[1])) for p in traj['waypoints'])
    else:
        planned_path = simulate_path(start, traj['segments'])
    planned_len = sum(
        math.hypot(planned_path[i + 1][0] - planned_path[i][0],
                    planned_path[i + 1][1] - planned_path[i][1])
        for i in range(len(planned_path) - 1)
    )

    if not skip_cleanup:
        _cleanup_webots()
    out_dir.mkdir(parents=True, exist_ok=True)
    gt_json = out_dir / f'{world}_gt.json'
    if gt_json.exists():
        gt_json.unlink()
    sim_log = out_dir / f'{world}_sim.log'
    drv_log = out_dir / f'{world}_driver.log'

    recorder = _record_ground_truth(gt_json, domain_id=domain_id)
    time.sleep(1.5)

    is_waypoint_traj = traj.get('type') == 'waypoints'
    if is_waypoint_traj:
        print(f'  launching {world} (waypoint, {len(traj["waypoints"])} pts)...',
              flush=True)
        sim_proc = _launch_headless_sim(world, domain_id, sim_log, mode=sim_mode,
                                          webots_port=webots_port)
        # Wait for sim to come up; the driver itself will spin until /odom arrives.
        time.sleep(20)
        # Cap runtime at 2x planned length / max_v + 60s slack.
        max_v = float(traj.get('max_v', 0.4))
        max_runtime = max(60.0, 2.0 * planned_len / max_v + 60.0)
        drv_proc = _run_waypoint_driver(traj_yaml_path, domain_id, drv_log, max_runtime)
        drv_proc.wait(timeout=max_runtime + 30)
        print(f'    driver exited code={drv_proc.returncode}', flush=True)
        time.sleep(3)  # settle
        _stop_proc(sim_proc)
    else:
        # legacy: open-loop via scripted_runner
        env = os.environ.copy()
        env.setdefault('PYTHONUNBUFFERED', '1')
        env['ROS_DOMAIN_ID'] = str(domain_id)
        env.setdefault('SIM_EXTRA_RTABMAP_ARGS',
                        '--Icp/VoxelSize 0.10 --Icp/MaxCorrespondenceDistance 0.30 '
                        '--Icp/PointToPlaneK 5 --Icp/PointToPlaneRadius 0 '
                        '--Reg/Force3DoF true')
        sim_out = out_dir / f'{world}_sim'
        print(f'  launching headless sim for {world} (traj={traj_name})...', flush=True)
        runner_proc = subprocess.Popen([
            'python3', '-u', '-m', 'rove_sim_webots.scripted_runner',
            '--mode', 'live', '--world', f'{world}.wbt', '--trajectory', traj_name,
            '--out-dir', str(sim_out), '--db-name', 'rtabmap.db',
            '--domain-id', str(domain_id), '--headless',
            '--post-drive-settle', '3',
        ], env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        while True:
            line = runner_proc.stdout.readline()
            if not line:
                break
            if 'wrote' in line or 'failed' in line.lower():
                print(f'    {line.strip()}', flush=True)
        runner_proc.wait()
        print(f'    runner exited code={runner_proc.returncode}', flush=True)

    _stop_recorder(recorder)
    if not skip_cleanup:
        _cleanup_webots()

    if not gt_json.exists():
        return VerifyResult(world, traj_name, planned_len, 0.0, 0,
                              0.0, 0.0, 0, 'no_data')

    raw = json.loads(gt_json.read_text())
    if not raw:
        return VerifyResult(world, traj_name, planned_len, 0.0, 0,
                              0.0, 0.0, 0, 'no_data')
    actual = [(p['x'], p['y']) for p in raw]
    actual_len = sum(
        math.hypot(actual[i + 1][0] - actual[i][0],
                    actual[i + 1][1] - actual[i][1])
        for i in range(len(actual) - 1)
    )

    # Min clearance from any obstacle along the actual path.
    # collision = robot center is INSIDE an obstacle's footprint (c < 0).
    # soft_fail = robot edge brushes within 10 cm of an obstacle edge
    #             (i.e., robot center within ROBOT_RADIUS - 0.10 of edge).
    min_clear = float('inf')
    collisions = 0
    SOFT_THRESH = ROBOT_RADIUS_M - 0.10
    for px, py in actual:
        c = min((o.clearance(px, py) for o in obstacles), default=float('inf'))
        if c < min_clear:
            min_clear = c
        if c < 0.0:
            collisions += 1

    max_dev = 0.0
    for px, py in actual:
        d = min(math.hypot(px - qx, py - qy) for qx, qy in planned_path)
        if d > max_dev:
            max_dev = d

    if collisions > 0:
        status = 'collision'
    elif min_clear < SOFT_THRESH:
        status = 'soft_fail'
    else:
        status = 'pass'

    return VerifyResult(world, traj_name, planned_len, actual_len, len(actual),
                          min_clear, max_dev, collisions, status)


def main():
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument('--out-dir', type=Path,
                    default=Path.home() / 'overnight_runs' / 'auto_traj_verify')
    p.add_argument('--worlds', nargs='*', default=None,
                    help='Subset of worlds to test; default = all.')
    p.add_argument('--domain-id', type=int, default=125)
    p.add_argument('--sim-mode', choices=('realtime', 'fast'), default='realtime',
                    help='Webots run mode passed to sim.launch.py.')
    args = p.parse_args()

    worlds = args.worlds or WORLDS
    args.out_dir.mkdir(parents=True, exist_ok=True)
    # When running multiple parallel verifiers, an instance must not run the
    # global webots-killer cleanup — that would kill sibling instances.
    skip_cleanup = bool(os.environ.get('VERIFY_SKIP_CLEANUP'))
    webots_port = int(os.environ.get('WEBOTS_PORT', '1234'))

    results = []
    timings = []
    for w in worlds:
        t0 = time.monotonic()
        try:
            r = verify_world(w, args.out_dir, domain_id=args.domain_id,
                              sim_mode=args.sim_mode,
                              webots_port=webots_port,
                              skip_cleanup=skip_cleanup)
        except Exception as e:
            print(f'!! {w} raised: {e!r}', flush=True)
            r = VerifyResult(w, '?', 0.0, 0.0, 0, 0.0, 0.0, 0, f'error:{e}')
        wall = time.monotonic() - t0
        timings.append((w, wall))
        results.append(r)
        print(f'  -> {w}: {r.status}  '
              f'planned={r.planned_length_m:.1f}m actual={r.actual_length_m:.1f}m '
              f'clear={r.min_actual_clearance_m:.2f}m dev={r.max_deviation_m:.2f}m '
              f'collisions={r.collisions}  wall={wall:.1f}s', flush=True)

    summary_path = args.out_dir / 'summary.json'
    summary_payload = []
    for r, (_w, wall) in zip(results, timings):
        d = asdict(r)
        d['wall_clock_s'] = wall
        summary_payload.append(d)
    summary_path.write_text(json.dumps({
        'sim_mode': args.sim_mode,
        'results': summary_payload,
    }, indent=2))
    print(f'\nwrote {summary_path}')
    n_pass = sum(1 for r in results if r.status == 'pass')
    n_soft = sum(1 for r in results if r.status == 'soft_fail')
    n_coll = sum(1 for r in results if r.status == 'collision')
    print(f'pass: {n_pass}/{len(results)}  soft_fail: {n_soft}  collision: {n_coll}')
    return 0 if n_coll == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
