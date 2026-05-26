"""Trial evaluator that replays a pre-recorded sim bag through rtabmap.

Cheaper than re-running Webots per trial. The bag must have been recorded
from `scripted_runner --mode record` (which captures /livox/lidar,
/livox/imu, /ground_truth/odom, /tf, /tf_static, /clock).

Per trial:
  1. Launch rtabmap_lidar3d.launch.py with this trial's ICP params.
  2. Start a bag recorder for /ground_truth/odom + /rtabmap/odom + /tf.
  3. ros2 bag play the sim bag — feeds lidar/imu to rtabmap and GT to
     the output recorder.
  4. Stop rtabmap + recorder when bag playback ends.
  5. Run validator on the captured output bag → drift_ratio + ATE.

Drift_ratio is the score the optimizer minimizes.
"""
from __future__ import annotations

import dataclasses
import json
import os
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

from rove_tuning_orchestrator.evaluators.base import EvaluationResult, Evaluator


@dataclass
class SimBagEvaluatorConfig:
    bag: Path
    frame_id: str = 'livox_frame'
    lidar_topic: str = '/livox/lidar'
    imu_topic: str = '/livox/imu'
    gt_topic: str = '/ground_truth/odom'
    domain_id: int = 122
    warmup_s: float = 4.0
    drain_s: float = 3.0
    timeout_s: float = 180.0
    # Hard cap on a single replay; protects against rtabmap hang.
    max_replay_duration_s: float = 350.0


class SimBagEvaluator(Evaluator):
    """Score a trial by replaying a sim bag and measuring drift_ratio vs GT."""

    def __init__(self, cfg: SimBagEvaluatorConfig):
        self.cfg = cfg
        if not Path(cfg.bag).exists():
            raise FileNotFoundError(f'sim bag not found: {cfg.bag}')

    def evaluate(
        self,
        params: Dict[str, Any],
        trial_id: str,
        out_dir: Path,
    ) -> EvaluationResult:
        out_dir.mkdir(parents=True, exist_ok=True)
        cfg = self.cfg
        db_path = out_dir / 'rtabmap.db'
        if db_path.exists():
            db_path.unlink()
        outbag_path = out_dir / 'eval_bag'
        if outbag_path.exists():
            import shutil
            shutil.rmtree(outbag_path)

        env = os.environ.copy()
        env['ROS_DOMAIN_ID'] = str(cfg.domain_id)
        env.setdefault('PYTHONUNBUFFERED', '1')

        param_args = []
        for k, v in params.items():
            if isinstance(v, bool):
                v = 'true' if v else 'false'
            param_args.extend([f'--{k}', str(v)])
        extra_args = ' '.join(param_args)
        rtabmap_cmd = [
            'ros2', 'launch', 'rove_sim_webots', 'rtabmap_lidar3d.launch.py',
            f'frame_id:={cfg.frame_id}',
            f'lidar_topic:={cfg.lidar_topic}',
            f'imu_topic:={cfg.imu_topic}',
            'use_sim_time:=true',
            f'database_path:={db_path}',
            f'rtabmap_args:={extra_args}',
        ]
        rtabmap_log = out_dir / 'rtabmap.log'
        rtabmap_proc = subprocess.Popen(
            rtabmap_cmd, env=env,
            stdout=open(rtabmap_log, 'wb'), stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )

        time.sleep(cfg.warmup_s)

        bag_log = out_dir / 'bag.log'
        record_cmd = [
            'ros2', 'bag', 'record',
            '-o', str(outbag_path),
            cfg.gt_topic, '/rtabmap/odom', '/tf', '/tf_static', '/clock',
        ]
        record_proc = subprocess.Popen(
            record_cmd, env=env,
            stdout=open(bag_log, 'wb'), stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
        time.sleep(1.5)

        play_log = out_dir / 'play.log'
        play_cmd = [
            'ros2', 'bag', 'play', str(cfg.bag),
            '--clock',
            '--topics', cfg.lidar_topic, cfg.imu_topic, cfg.gt_topic,
            '/tf', '/tf_static',
        ]
        play_proc = subprocess.Popen(
            play_cmd, env=env,
            stdout=open(play_log, 'wb'), stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )

        failed = False
        failure_reason = ''
        try:
            play_proc.wait(timeout=cfg.max_replay_duration_s)
        except subprocess.TimeoutExpired:
            failure_reason = 'bag playback exceeded max_replay_duration_s'
            failed = True
            _kill_group(play_proc, timeout=5)
        time.sleep(cfg.drain_s)
        _kill_group(record_proc)
        _kill_group(rtabmap_proc)

        # Validator post-process
        try:
            from rove_sim_webots import validator as _val
            result = _val.validate(bag_path=outbag_path, gt_topic=cfg.gt_topic)
            drift_ratio = result.drift_ratio
            ate = result.ate_rmse_m
            traj_len = result.trajectory_length_m
            n_pairs = result.n_pairs
            n_gt_poses = result.n_gt_poses
            warnings = list(result.warnings)
        except Exception as e:
            return EvaluationResult(
                score=float('inf'), failed=True,
                failure_reason=f'validator raised: {e!r}',
                artifacts={'rtabmap_log': str(rtabmap_log),
                            'play_log': str(play_log), 'bag_log': str(bag_log),
                            'out_bag': str(outbag_path)},
                metrics={},
            )

        if drift_ratio is None or drift_ratio != drift_ratio:  # NaN
            failed = True
            failure_reason = failure_reason or 'drift_ratio is NaN/missing'

        metrics = {
            'drift_ratio': drift_ratio,
            'ate_rmse_m': ate,
            'trajectory_length_m': traj_len,
            'n_pairs': n_pairs,
            'n_gt_poses': n_gt_poses,
        }
        (out_dir / 'validation.json').write_text(
            json.dumps(metrics, indent=2)
        )
        return EvaluationResult(
            score=float(drift_ratio) if drift_ratio is not None else float('inf'),
            failed=failed,
            failure_reason=failure_reason,
            artifacts={
                'rtabmap_log': str(rtabmap_log),
                'play_log': str(play_log),
                'bag_log': str(bag_log),
                'out_bag': str(outbag_path),
                'db_path': str(db_path),
                'validation_json': str(out_dir / 'validation.json'),
            },
            metrics=metrics,
            warnings=warnings,
        )


def _kill_group(proc: subprocess.Popen, timeout: float = 10.0) -> None:
    import signal
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
