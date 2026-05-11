"""Run RTAB-Map trials over one or more rosbags.

A *trial* is one set of RTAB-Map parameter overrides applied to N rosbags.
Each bag gets its own ``ros2 launch`` + ``ros2 bag play`` pair and produces
its own ``rtabmap.db``. The runs are independent — concatenating bags into
one map doesn't help the loop-closure-drift metric we score on.

Outputs (rooted at ``<output-root>/<trial-id>/``):

    params.json         effective parameters used (DEFAULTS ∪ overrides)
    env.json            environmental config (topics, frames, qos, …)
    trial.json          aggregated TrialResult
    <bag-name>/
        trial.launch.py   rendered launch file
        launch.cmd        resolved ``ros2 launch`` command (for re-running by hand)
        bag.cmd           resolved ``ros2 bag play`` command
        launch.log        merged stdout/stderr of the launch subprocess
        bag.log           merged stdout/stderr of bag playback
        rtabmap.db        the resulting database (if the run succeeded)
        result.json       BagRunResult
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import os
import signal
import subprocess
import time
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Optional

from .scoring import score_run
from .template_renderer import effective_params, render


@dataclass
class EnvConfig:
    """Environmental settings that stay constant across a trial."""
    lidar_topic: str = '/livox/lidar'
    imu_topic: str = '/imu/data'
    frame_id: str = 'base_link'
    fixed_frame_id: str = ''
    expected_update_rate: float = 15.0
    qos: int = 1
    deskewing: bool = True
    bag_play_args: list[str] = field(default_factory=list)


@dataclass
class BagRunResult:
    bag_path: str
    db_path: Optional[str]
    success: bool
    duration_s: float
    failure_reason: Optional[str] = None
    metrics: Optional[dict] = None  # populated by scoring.score_run on success


@dataclass
class TrialResult:
    trial_id: str
    params: dict
    env: dict
    runs: list[BagRunResult]
    success: bool


def _build_launch_cmd(launch_file: Path, db_path: Path, env: EnvConfig) -> list[str]:
    args: dict[str, str] = {
        'database_path': str(db_path),
        'lidar_topic': env.lidar_topic,
        'imu_topic': env.imu_topic,
        'frame_id': env.frame_id,
        'fixed_frame_id': env.fixed_frame_id,
        'expected_update_rate': str(env.expected_update_rate),
        'qos': str(env.qos),
        'deskewing': str(env.deskewing).lower(),
        'use_sim_time': 'true',
        'enable_viz': 'false',
    }
    # `ros2 launch` rejects 'name:=' with an empty value; omit those args so the
    # launch file's declared default kicks in.
    return ['ros2', 'launch', str(launch_file)] + [
        f'{k}:={v}' for k, v in args.items() if v != ''
    ]


def _build_bag_cmd(bag_path: Path, env: EnvConfig) -> list[str]:
    return ['ros2', 'bag', 'play', str(bag_path), '--clock'] + list(env.bag_play_args)


def _terminate_process_group(proc: subprocess.Popen, sigint_timeout_s: float) -> None:
    """Send SIGINT to the launch's process group; escalate to SIGTERM then
    SIGKILL if it doesn't exit. Safe to call on an already-dead process.
    """
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        return

    escalation = [
        (signal.SIGINT, sigint_timeout_s),
        (signal.SIGTERM, 5.0),
        (signal.SIGKILL, 5.0),
    ]
    for sig, timeout in escalation:
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            return
        try:
            proc.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            continue


def run_single_bag(
    bag_path: Path,
    overrides: dict,
    output_dir: Path,
    env: EnvConfig,
    *,
    warmup_s: float = 5.0,
    drain_s: float = 3.0,
    shutdown_timeout_s: float = 30.0,
    max_bag_duration_s: Optional[float] = None,
    ros_domain_id: Optional[int] = None,
    dry_run: bool = False,
) -> BagRunResult:
    output_dir.mkdir(parents=True, exist_ok=True)
    launch_file = output_dir / 'trial.launch.py'
    db_path = output_dir / 'rtabmap.db'

    render(overrides, output_path=launch_file)

    launch_cmd = _build_launch_cmd(launch_file, db_path, env)
    bag_cmd = _build_bag_cmd(bag_path, env)

    # Build the subprocess environment. When ros_domain_id is provided, override
    # ROS_DOMAIN_ID so multiple concurrent trials don't see each other's topics
    # (essential for parallel execution).
    proc_env = os.environ.copy()
    if ros_domain_id is not None:
        proc_env['ROS_DOMAIN_ID'] = str(ros_domain_id)

    (output_dir / 'launch.cmd').write_text(
        (f'ROS_DOMAIN_ID={ros_domain_id} ' if ros_domain_id is not None else '')
        + ' '.join(launch_cmd) + '\n'
    )
    (output_dir / 'bag.cmd').write_text(' '.join(bag_cmd) + '\n')

    if dry_run:
        return BagRunResult(
            bag_path=str(bag_path),
            db_path=None,
            success=True,
            duration_s=0.0,
            failure_reason='dry-run',
        )

    if not bag_path.exists():
        return BagRunResult(
            bag_path=str(bag_path),
            db_path=None,
            success=False,
            duration_s=0.0,
            failure_reason=f'bag path does not exist: {bag_path}',
        )

    launch_log_path = output_dir / 'launch.log'
    bag_log_path = output_dir / 'bag.log'
    launch_proc: Optional[subprocess.Popen] = None
    start = time.time()

    try:
        with open(launch_log_path, 'w') as launch_log:
            launch_proc = subprocess.Popen(
                launch_cmd,
                stdout=launch_log,
                stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL,
                start_new_session=True,
                env=proc_env,
            )

            time.sleep(warmup_s)

            if launch_proc.poll() is not None:
                return BagRunResult(
                    bag_path=str(bag_path),
                    db_path=None,
                    success=False,
                    duration_s=time.time() - start,
                    failure_reason=(
                        f'launch exited before bag started '
                        f'(code {launch_proc.returncode}); see {launch_log_path}'
                    ),
                )

            bag_timed_out = False
            with open(bag_log_path, 'w') as bag_log:
                bag_proc = subprocess.Popen(
                    bag_cmd,
                    stdout=bag_log,
                    stderr=subprocess.STDOUT,
                    stdin=subprocess.DEVNULL,
                    start_new_session=True,
                    env=proc_env,
                )
                try:
                    bag_proc.wait(timeout=max_bag_duration_s)
                except subprocess.TimeoutExpired:
                    bag_timed_out = True
                    _terminate_process_group(bag_proc, sigint_timeout_s=10.0)
            # Build a stand-in result so the rest of the function can stay generic.
            bag_returncode = bag_proc.returncode

            time.sleep(drain_s)

        # Outside the launch_log context: SIGINT the launch group and wait
        # for it to flush rtabmap.db on shutdown.
        _terminate_process_group(launch_proc, shutdown_timeout_s)
        launch_proc = None

        duration = time.time() - start

        if bag_timed_out:
            return BagRunResult(
                bag_path=str(bag_path),
                db_path=str(db_path) if db_path.exists() else None,
                success=False,
                duration_s=duration,
                failure_reason=(
                    f'bag playback exceeded --max-bag-duration-s '
                    f'({max_bag_duration_s}s); see {bag_log_path}'
                ),
            )

        if bag_returncode != 0:
            return BagRunResult(
                bag_path=str(bag_path),
                db_path=str(db_path) if db_path.exists() else None,
                success=False,
                duration_s=duration,
                failure_reason=f'bag exited with code {bag_returncode}; see {bag_log_path}',
            )

        if not db_path.exists():
            return BagRunResult(
                bag_path=str(bag_path),
                db_path=None,
                success=False,
                duration_s=duration,
                failure_reason=f'rtabmap.db was not created; see {launch_log_path}',
            )

        score = score_run(db_path, output_dir)
        return BagRunResult(
            bag_path=str(bag_path),
            db_path=str(db_path),
            success=True,
            duration_s=duration,
            metrics=asdict(score),
        )
    finally:
        if launch_proc is not None and launch_proc.poll() is None:
            _terminate_process_group(launch_proc, shutdown_timeout_s)


def run_trial(
    trial_id: str,
    overrides: dict,
    bags: list[Path],
    output_root: Path,
    env: EnvConfig,
    *,
    dry_run: bool = False,
    warmup_s: float = 5.0,
    drain_s: float = 3.0,
    shutdown_timeout_s: float = 30.0,
    max_bag_duration_s: Optional[float] = None,
    ros_domain_id: Optional[int] = None,
) -> TrialResult:
    trial_dir = output_root / trial_id
    trial_dir.mkdir(parents=True, exist_ok=True)

    eff = effective_params(overrides)
    (trial_dir / 'params.json').write_text(json.dumps(eff, indent=2, sort_keys=True))
    (trial_dir / 'env.json').write_text(json.dumps(asdict(env), indent=2, sort_keys=True))

    # Avoid two bags colliding when they share a stem (e.g. /a/foo and /b/foo).
    used_names: dict[str, int] = {}
    runs: list[BagRunResult] = []
    for bag in bags:
        name = bag.stem or bag.name
        if name in used_names:
            used_names[name] += 1
            name = f'{name}_{used_names[name]}'
        else:
            used_names[name] = 0

        bag_dir = trial_dir / name
        print(f'[trial {trial_id}] bag {bag} -> {bag_dir}')
        result = run_single_bag(
            bag, overrides, bag_dir, env,
            warmup_s=warmup_s, drain_s=drain_s,
            shutdown_timeout_s=shutdown_timeout_s,
            max_bag_duration_s=max_bag_duration_s,
            ros_domain_id=ros_domain_id,
            dry_run=dry_run,
        )
        (bag_dir / 'result.json').write_text(json.dumps(asdict(result), indent=2, sort_keys=True))
        runs.append(result)
        status = 'OK' if result.success else 'FAIL'
        suffix = f' — {result.failure_reason}' if result.failure_reason else ''
        print(f'[trial {trial_id}] bag {name}: {status} ({result.duration_s:.1f}s){suffix}')

    trial_result = TrialResult(
        trial_id=trial_id,
        params=eff,
        env=asdict(env),
        runs=runs,
        success=all(r.success for r in runs),
    )
    (trial_dir / 'trial.json').write_text(
        json.dumps(asdict(trial_result), indent=2, sort_keys=True)
    )
    return trial_result


def _parse_bool(s: str) -> bool:
    return s.strip().lower() in ('true', '1', 'yes', 'y')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        '--bag', '-b', type=Path, action='append', required=True, dest='bags',
        help='Path to a rosbag (ROS 2 bag directory). Repeat for multiple bags.',
    )
    parser.add_argument(
        '--output-root', '-o', type=Path, required=True,
        help='Root directory for trial outputs. Each trial gets its own subdir.',
    )
    parser.add_argument(
        '--trial-id', default=None,
        help='Trial identifier (becomes a subdir name). Default: trial_<timestamp>.',
    )
    parser.add_argument(
        '--set', '-s', action='append', default=[], metavar='KEY=VALUE', dest='overrides',
        help='Param override. Repeat for multiple. Defaults apply to unset keys.',
    )
    parser.add_argument('--lidar-topic', default='/livox/lidar')
    parser.add_argument('--imu-topic', default='/imu/data')
    parser.add_argument('--frame-id', default='base_link')
    parser.add_argument('--fixed-frame-id', default='')
    parser.add_argument('--expected-update-rate', type=float, default=15.0)
    parser.add_argument('--qos', type=int, default=1)
    parser.add_argument('--deskewing', type=_parse_bool, default=True)
    parser.add_argument(
        '--bag-play-arg', action='append', default=[],
        help='Extra arg passed to `ros2 bag play` (after `--clock`). Repeat for multiple.',
    )
    parser.add_argument('--warmup-s', type=float, default=5.0)
    parser.add_argument('--drain-s', type=float, default=3.0)
    parser.add_argument('--shutdown-timeout-s', type=float, default=30.0)
    parser.add_argument(
        '--max-bag-duration-s', type=float, default=None,
        help='Wall-clock cap on bag playback; SIGINTs the bag if exceeded. '
             'Useful as a safety net against corrupt/runaway bags.',
    )
    parser.add_argument(
        '--dry-run', action='store_true',
        help='Render launch files and write resolved commands; do not actually run anything.',
    )
    args = parser.parse_args()

    overrides: dict[str, str] = {}
    for entry in args.overrides:
        if '=' not in entry:
            parser.error(f'--set expects KEY=VALUE, got: {entry!r}')
        key, _, value = entry.partition('=')
        overrides[key] = value

    trial_id = args.trial_id or time.strftime('trial_%Y%m%d_%H%M%S')

    env = EnvConfig(
        lidar_topic=args.lidar_topic,
        imu_topic=args.imu_topic,
        frame_id=args.frame_id,
        fixed_frame_id=args.fixed_frame_id,
        expected_update_rate=args.expected_update_rate,
        qos=args.qos,
        deskewing=args.deskewing,
        bag_play_args=args.bag_play_arg,
    )

    result = run_trial(
        trial_id=trial_id,
        overrides=overrides,
        bags=args.bags,
        output_root=args.output_root,
        env=env,
        dry_run=args.dry_run,
        warmup_s=args.warmup_s,
        drain_s=args.drain_s,
        shutdown_timeout_s=args.shutdown_timeout_s,
        max_bag_duration_s=args.max_bag_duration_s,
    )

    succeeded = sum(r.success for r in result.runs)
    print()
    print(f'Trial {trial_id}: {"OK" if result.success else "FAIL"} '
          f'({succeeded}/{len(result.runs)} bags succeeded)')
    print(f'Output: {args.output_root / trial_id}')
    return 0 if result.success else 1


if __name__ == '__main__':
    raise SystemExit(main())
