"""End-to-end automation: launch sim, drive a trajectory, capture an RTAB-Map db.

Three modes:
- live:     sim + rtabmap together, db written directly to ~/.ros/<db_name>.
- record:   sim only, bag recorded to <out_dir>/<name>.bag/.
- validate: sim + rtabmap + ground-truth recording + post-run trajectory
            comparison. Produces a bag, a db, and a validation.json with ATE
            and drift metrics (RTAB-Map estimate vs Webots supervisor truth).

All modes set ROS_DOMAIN_ID inside [120, 140]; see feedback-ros-domain-range.
"""

from __future__ import annotations

import argparse
import os
import random
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import List, Optional


DOMAIN_RANGE = (120, 140)


def _pick_domain_id(explicit: Optional[int]) -> int:
    if explicit is not None:
        if not (DOMAIN_RANGE[0] <= explicit <= DOMAIN_RANGE[1]):
            raise SystemExit(
                f'--domain-id must be in [{DOMAIN_RANGE[0]}, {DOMAIN_RANGE[1]}], got {explicit}'
            )
        return explicit
    # Default: 130. Random within range only if the caller forces it.
    return 130


def _build_env(domain_id: int, headless: bool = False) -> dict:
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = str(domain_id)
    if headless:
        env['WEBOTS_GUI'] = 'false'
        # Force a clean isolated display via xvfb-run regardless of the real
        # $DISPLAY — keeps Webots from popping a "No Rendering" window on the
        # user's actual desktop during autonomous runs.
        env.pop('DISPLAY', None)
        env.pop('WAYLAND_DISPLAY', None)
    return env


def _clean_stale_xvfb_locks() -> None:
    """Remove /tmp/.Xn-lock + /tmp/.X11-unix/Xn for displays without a live
    Xvfb process. xvfb-run -a picks the lowest free display number; stale
    locks make it bump higher each call until it sometimes hits a number
    where Xvfb fails to come up (intermittent "X11 connection broke" at
    Webots startup). Cleaning before each trial keeps display numbers low
    and predictable.
    """
    import glob
    import subprocess as _sp
    for lock in glob.glob('/tmp/.X*-lock'):
        name = os.path.basename(lock)
        if not name.startswith('.X') or not name.endswith('-lock'):
            continue
        num = name[2:-5]
        if not num.isdigit():
            continue
        # If Xvfb is actually running on this display, skip it.
        rc = _sp.run(
            ['pgrep', '-f', f'Xvfb :{num} '], capture_output=True,
        ).returncode
        if rc == 0:
            continue
        try:
            os.remove(lock)
            sock = f'/tmp/.X11-unix/X{num}'
            if os.path.exists(sock):
                os.remove(sock)
        except OSError:
            pass


def _wrap_for_headless(cmd: List[str], headless: bool) -> List[str]:
    """In headless mode, always prepend xvfb-run for a clean isolated display.

    Previously this only wrapped when $DISPLAY was unset, which meant runs
    on workstations with a real X server would pop a Webots window. The new
    behaviour ALWAYS wraps when headless=True so no GUI ever appears.
    """
    if not headless:
        return cmd
    if shutil.which('xvfb-run') is None:
        raise SystemExit(
            'Headless mode requested but xvfb-run is not on PATH. '
            'Install xvfb (apt install xvfb).'
        )
    _clean_stale_xvfb_locks()
    return ['xvfb-run', '-a', '--server-args=-screen 0 1024x768x24', *cmd]


def _wait_for_topic(env: dict, topic: str, timeout_s: float) -> bool:
    """Block until `topic` shows up in `ros2 topic list` or timeout expires."""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            out = subprocess.check_output(
                ['ros2', 'topic', 'list'], env=env, text=True, timeout=5,
            )
        except subprocess.SubprocessError:
            out = ''
        if topic in out.splitlines():
            return True
        time.sleep(1.0)
    return False


def _drive_trajectory(env: dict, trajectory_path: Path) -> None:
    """Publish cmd_vel segments using rclpy in this process."""
    import rclpy
    from geometry_msgs.msg import Twist

    from rove_sim_webots.trajectories import Trajectory

    traj = Trajectory.load(trajectory_path)

    rclpy.init()
    node = rclpy.create_node('rove_sim_trajectory_driver')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)  # absolute — avoid namespace surprises
    log = node.get_logger()

    log.info(f'Driving "{traj.name}" — {traj.duration:.1f} s, {len(traj.segments)} segments')

    # Wait until the rove driver is actually subscribed. Without this, on a
    # loaded box DDS discovery can take >1s and we'd burn through the first
    # segment publishing into the void.
    sub_wait_deadline = time.time() + 10.0
    while time.time() < sub_wait_deadline:
        if pub.get_subscription_count() >= 1:
            log.info(f'  cmd_vel subscriber discovered ({pub.get_subscription_count()})')
            break
        time.sleep(0.1)
    else:
        log.warn('  no /cmd_vel subscriber found after 10s — publishing anyway')

    for i, seg in enumerate(traj.segments):
        twist = Twist()
        twist.linear.x = seg.v
        twist.angular.z = seg.w
        log.info(f'  [{i+1}/{len(traj.segments)}] v={seg.v:+.2f} w={seg.w:+.2f} for {seg.dt:.2f}s')
        end = time.time() + seg.dt
        while time.time() < end:
            pub.publish(twist)
            time.sleep(0.05)  # 20 Hz cmd_vel

    # Stop.
    stop = Twist()
    for _ in range(20):
        pub.publish(stop)
        time.sleep(0.05)

    node.destroy_node()
    rclpy.shutdown()


def _resolve_trajectory(arg: str) -> Path:
    p = Path(arg)
    if p.exists():
        return p
    # Try package share.
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('rove_sim_webots'))
        candidate = share / 'config' / 'trajectories' / (arg if arg.endswith('.yaml') else f'{arg}.yaml')
        if candidate.exists():
            return candidate
    except Exception:
        pass
    raise SystemExit(f'Trajectory not found: {arg}')


def _start_subprocess(cmd: List[str], env: dict, log_path: Optional[Path] = None) -> subprocess.Popen:
    log_f = open(log_path, 'wb') if log_path else subprocess.DEVNULL
    return subprocess.Popen(
        cmd,
        env=env,
        stdout=log_f,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )


def _kill_group(proc: subprocess.Popen, timeout: float = 10.0) -> None:
    if proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        try:
            proc.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            pass
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        try:
            proc.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            pass
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except ProcessLookupError:
        pass


def run_live(args) -> int:
    """sim + rtabmap together; db is written directly to ~/.ros/<db_name>."""
    env = _build_env(args.domain_id, headless=args.headless)
    out_dir = Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    db_path = Path('~/.ros').expanduser() / args.db_name
    if db_path.exists() and not args.append:
        db_path.unlink()
        print(f'[scripted_run] cleared existing db at {db_path}')

    sim_log = out_dir / 'sim.log'
    print(f'[scripted_run] ROS_DOMAIN_ID={env["ROS_DOMAIN_ID"]} headless={args.headless}')
    print(f'[scripted_run] starting sim+rtabmap, logs -> {sim_log}')
    sim = _start_subprocess(
        _wrap_for_headless(
            ['ros2', 'launch', 'rove_sim_webots', 'sim_with_rtabmap.launch.py',
             f'world:={args.world}', f'db_name:={args.db_name}'],
            args.headless,
        ),
        env=env, log_path=sim_log,
    )

    try:
        if not _wait_for_topic(env, '/livox/lidar', timeout_s=60):
            print('[scripted_run] timed out waiting for /livox/lidar', file=sys.stderr)
            return 2
        trajectory_path = _resolve_trajectory(args.trajectory)
        _drive_trajectory(env, trajectory_path)
        # Settle so rtabmap finalizes the db.
        time.sleep(args.post_drive_settle)
    finally:
        print('[scripted_run] stopping sim+rtabmap')
        _kill_group(sim)

    final = out_dir / args.db_name
    if db_path.exists():
        shutil.copy(db_path, final)
        print(f'[scripted_run] wrote {final}')
    else:
        print(f'[scripted_run] WARNING: no db produced at {db_path}', file=sys.stderr)
        return 3
    return 0


def run_validate(args) -> int:
    """sim + rtabmap + ground-truth bag + post-run trajectory comparison."""
    env = _build_env(args.domain_id, headless=args.headless)
    out_dir = Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    bag_path = out_dir / args.bag_name
    db_path = Path('~/.ros').expanduser() / args.db_name
    if db_path.exists() and not args.append:
        db_path.unlink()

    sim_log = out_dir / 'sim.log'
    bag_log = out_dir / 'bag.log'
    print(f'[scripted_run] ROS_DOMAIN_ID={env["ROS_DOMAIN_ID"]} headless={args.headless}')
    print(f'[scripted_run] starting sim+rtabmap, logs -> {sim_log}')
    sim = _start_subprocess(
        _wrap_for_headless(
            ['ros2', 'launch', 'rove_sim_webots', 'sim_with_rtabmap.launch.py',
             f'world:={args.world}', f'db_name:={args.db_name}'],
            args.headless,
        ),
        env=env, log_path=sim_log,
    )

    bag = None
    try:
        if not _wait_for_topic(env, '/livox/lidar', timeout_s=60):
            print('[scripted_run] timed out waiting for /livox/lidar', file=sys.stderr)
            return 2
        # Also wait for ground truth so we never produce a bag without it.
        if not _wait_for_topic(env, '/ground_truth/odom', timeout_s=30):
            print('[scripted_run] timed out waiting for /ground_truth/odom — '
                  'is supervisor=TRUE in Rove.proto?', file=sys.stderr)
            return 2

        topics = [
            '/livox/lidar', '/livox/imu',
            '/ground_truth/odom',
            '/rtabmap/odom', '/rtabmap/mapPath',
            '/odom', '/tf', '/tf_static', '/clock',
            '/cmd_vel',  # debug — confirms trajectory_driver actually publishes
        ]
        print(f'[scripted_run] recording bag -> {bag_path}')
        bag = _start_subprocess(
            ['ros2', 'bag', 'record', '-o', str(bag_path), *topics],
            env=env, log_path=bag_log,
        )
        time.sleep(2.0)

        trajectory_path = _resolve_trajectory(args.trajectory)
        _drive_trajectory(env, trajectory_path)
        time.sleep(args.post_drive_settle)
    finally:
        if bag is not None:
            print('[scripted_run] stopping bag recorder')
            _kill_group(bag, timeout=15)
        print('[scripted_run] stopping sim+rtabmap')
        _kill_group(sim)

    if not (bag_path / 'metadata.yaml').exists():
        print(f'[scripted_run] WARNING: no bag metadata at {bag_path}', file=sys.stderr)
        return 3

    # Copy the produced db next to the bag if it landed in ~/.ros.
    if db_path.exists():
        import shutil as _sh
        _sh.copy(db_path, out_dir / args.db_name)

    # Run the validator post-process.
    print('[scripted_run] running post-run validator')
    from rove_sim_webots import validator
    result = validator.validate(bag_path=bag_path)
    (out_dir / 'validation.json').write_text(
        __import__('json').dumps(__import__('dataclasses').asdict(result), indent=2)
    )
    print(__import__('json').dumps(__import__('dataclasses').asdict(result), indent=2))
    print(f"[scripted_run] validation.json -> {out_dir / 'validation.json'}")
    return 0


def run_record(args) -> int:
    """sim only; ros2 bag record captures topics for later processing."""
    env = _build_env(args.domain_id, headless=args.headless)
    out_dir = Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    bag_path = out_dir / args.bag_name

    sim_log = out_dir / 'sim.log'
    bag_log = out_dir / 'bag.log'
    print(f'[scripted_run] ROS_DOMAIN_ID={env["ROS_DOMAIN_ID"]} headless={args.headless}')
    print(f'[scripted_run] starting sim, logs -> {sim_log}')
    sim = _start_subprocess(
        _wrap_for_headless(
            ['ros2', 'launch', 'rove_sim_webots', 'sim.launch.py', f'world:={args.world}'],
            args.headless,
        ),
        env=env, log_path=sim_log,
    )

    bag = None
    try:
        if not _wait_for_topic(env, '/livox/lidar', timeout_s=60):
            print('[scripted_run] timed out waiting for /livox/lidar', file=sys.stderr)
            return 2

        topics = [
            '/livox/lidar', '/livox/imu', '/rove/camera/image_raw',
            '/rove/gps', '/odom', '/tf', '/tf_static', '/clock',
        ]
        print(f'[scripted_run] recording bag -> {bag_path}')
        bag = _start_subprocess(
            ['ros2', 'bag', 'record', '-o', str(bag_path), *topics],
            env=env, log_path=bag_log,
        )
        time.sleep(2.0)

        trajectory_path = _resolve_trajectory(args.trajectory)
        _drive_trajectory(env, trajectory_path)
        time.sleep(args.post_drive_settle)
    finally:
        if bag is not None:
            print('[scripted_run] stopping bag recorder')
            _kill_group(bag, timeout=15)
        print('[scripted_run] stopping sim')
        _kill_group(sim)

    if (bag_path / 'metadata.yaml').exists():
        print(f'[scripted_run] bag written: {bag_path}')
        return 0
    print(f'[scripted_run] WARNING: no bag metadata at {bag_path}', file=sys.stderr)
    return 3


def main() -> int:
    p = argparse.ArgumentParser(prog='scripted_run')
    p.add_argument('--mode', choices=('live', 'record', 'validate'), default='live',
                   help='live: sim+rtabmap -> db; record: sim only -> bag; '
                        'validate: sim+rtabmap+gt -> bag+db+validation.json.')
    p.add_argument('--world', default='outdoor_terrain.wbt',
                   help='World file under share/rove_sim_webots/worlds/.')
    p.add_argument('--trajectory', default='outdoor_loop1',
                   help='Trajectory name or path (without .yaml the name is resolved from share/).')
    p.add_argument('--out-dir', default='./sim_runs/run_$(date +%%Y%%m%%d_%%H%%M%%S)',
                   help='Output directory for logs / bag / db copy.')
    p.add_argument('--db-name', default='sim.db', help='[live] db filename.')
    p.add_argument('--bag-name', default='sim_bag', help='[record] bag dir name.')
    p.add_argument('--append', action='store_true',
                   help='[live] do not clear an existing db before launching.')
    p.add_argument('--domain-id', type=int, default=None,
                   help=f'ROS_DOMAIN_ID; must be in [{DOMAIN_RANGE[0]}, {DOMAIN_RANGE[1]}] (default 130).')
    p.add_argument('--post-drive-settle', type=float, default=4.0,
                   help='Seconds to keep the sim running after the trajectory ends.')
    p.add_argument('--headless', action='store_true',
                   help='Run Webots without rendering (sets WEBOTS_GUI=false, '
                        'wraps with xvfb-run if no $DISPLAY). Required for server '
                        'autonomous runs.')
    args = p.parse_args()

    # Replace $(date ...) in the default out_dir with an actual timestamp.
    if '$(date' in args.out_dir:
        args.out_dir = time.strftime('./sim_runs/run_%Y%m%d_%H%M%S')

    args.domain_id = _pick_domain_id(args.domain_id)

    # Critical: also export the domain ID into THIS process's env, so the
    # trajectory driver's rclpy.init() picks it up. Otherwise the runner's
    # publisher is on domain 0 while the sim subprocess is on domain N, and
    # nothing connects.
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)

    if args.mode == 'live':
        return run_live(args)
    if args.mode == 'validate':
        return run_validate(args)
    return run_record(args)


if __name__ == '__main__':
    sys.exit(main())
