"""SimEvaluator — runs the Webots sim with --mode validate and returns ATE.

Wraps rove_sim_webots.scripted_runner. The simulator's supervisor provides
ground truth, so we get a real ATE metric (not the loop-closure-drift proxy
the real-data tuner currently uses).

Score = drift_ratio (final pose error / trajectory length), lower-is-better.
Tracking-loss penalty if RTAB-Map produces too few estimated poses.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

from rove_tuning_orchestrator.evaluators.base import EvaluationResult, Evaluator


@dataclass
class SimEvaluatorConfig:
    world: str = 'outdoor_terrain.wbt'
    trajectory: str = 'outdoor_loop1'
    headless: bool = True
    domain_id: int = 122  # in 120-140 range; orchestrator may override per-worker
    # Per-instance Webots IPC port. MUST be unique across concurrent sims on
    # the same machine. WebotsLauncher and WebotsController are coupled through
    # this (the controller connects to <port>'s IPC socket).
    webots_port: int = 1234
    timeout_s: float = 300.0
    # Penalty added to score per "missing estimated pose" — discourages
    # tracking-loss without making the metric unbounded.
    tracking_loss_penalty: float = 0.001
    # If fewer than this fraction of GT poses get a matched estimate,
    # the trial is marked failed (tracking effectively lost).
    min_pair_ratio: float = 0.30
    # If the robot moved less than this many meters (per ground truth),
    # the trial is marked failed. Defeats "robot didn't move => low drift_ratio"
    # gaming where the scorer sees small_drift / small_traj as success.
    # See project-scoring-gaming memory for the precedent.
    min_trajectory_length_m: float = 1.0


class SimEvaluator(Evaluator):
    name = 'sim'

    def __init__(self, cfg: Optional[SimEvaluatorConfig] = None):
        self.cfg = cfg or SimEvaluatorConfig()

    def evaluate(
        self,
        params: Dict[str, Any],
        trial_id: str,
        out_dir: Path,
    ) -> EvaluationResult:
        out_dir.mkdir(parents=True, exist_ok=True)

        # Translate RTAB-Map params dict into the --args string the
        # sim_with_rtabmap launch consumes.
        rtabmap_args = _params_to_rtabmap_args(params)

        # Pass via env var so we don't have to plumb a new launch arg;
        # sim_with_rtabmap.launch.py reads SIM_EXTRA_RTABMAP_ARGS (see launch
        # file). The orchestrator-side fallback is to set this here.
        env = os.environ.copy()
        env['SIM_EXTRA_RTABMAP_ARGS'] = rtabmap_args
        env['ROS_DOMAIN_ID'] = str(self.cfg.domain_id)
        env['WEBOTS_PORT'] = str(self.cfg.webots_port)

        cmd = [
            'python3', '-m', 'rove_sim_webots.scripted_runner',
            '--mode', 'validate',
            '--world', self.cfg.world,
            '--trajectory', self.cfg.trajectory,
            '--out-dir', str(out_dir),
            '--bag-name', 'bag',
            '--db-name', 'rtabmap.db',
            '--domain-id', str(self.cfg.domain_id),
        ]
        if self.cfg.headless:
            cmd.append('--headless')

        log_path = out_dir / 'runner.log'
        try:
            with open(log_path, 'wb') as lf:
                subprocess.run(
                    cmd, env=env, stdout=lf, stderr=subprocess.STDOUT,
                    timeout=self.cfg.timeout_s, check=False,
                )
        except subprocess.TimeoutExpired:
            return EvaluationResult(
                score=float('inf'),
                failed=True,
                failure_reason=f'sim run exceeded {self.cfg.timeout_s}s',
                artifacts={'runner_log': str(log_path)},
            )

        # The runner writes validation.json on success.
        val_json = out_dir / 'validation.json'
        if not val_json.exists():
            return EvaluationResult(
                score=float('inf'),
                failed=True,
                failure_reason='sim run did not produce validation.json',
                artifacts={'runner_log': str(log_path)},
            )

        report = json.loads(val_json.read_text())
        return self._result_from_report(report, out_dir, log_path)

    def _result_from_report(
        self, report: Dict[str, Any], out_dir: Path, log_path: Path,
    ) -> EvaluationResult:
        n_gt = report.get('n_gt_poses', 0)
        n_pairs = report.get('n_pairs', 0)
        pair_ratio = (n_pairs / n_gt) if n_gt > 0 else 0.0

        drift_ratio = report.get('drift_ratio')
        ate_rmse = report.get('ate_rmse_m')

        if drift_ratio is None or ate_rmse is None:
            return EvaluationResult(
                score=float('inf'),
                failed=True,
                failure_reason=(
                    f'validator reported no metrics (n_pairs={n_pairs}, '
                    f'n_gt={n_gt}) — likely RTAB-Map produced no estimate'
                ),
                metrics=report,
                artifacts={
                    'runner_log': str(log_path),
                    'validation_json': str(out_dir / 'validation.json'),
                    'bag': str(out_dir / 'bag'),
                },
            )

        traj_len = report.get('trajectory_length_m', 0.0) or 0.0

        if traj_len < self.cfg.min_trajectory_length_m:
            return EvaluationResult(
                score=2.0,  # worse than any valid drift_ratio + penalty
                failed=True,
                failure_reason=(
                    f'trajectory_length_m={traj_len:.3f} < '
                    f'{self.cfg.min_trajectory_length_m} — robot did not move; '
                    f'scoring would be gameable, rejecting trial'
                ),
                metrics={**report, 'pair_ratio': pair_ratio},
                artifacts={
                    'runner_log': str(log_path),
                    'validation_json': str(out_dir / 'validation.json'),
                },
            )

        if pair_ratio < self.cfg.min_pair_ratio:
            return EvaluationResult(
                score=drift_ratio + 1.0,  # heavy penalty but not infinite
                failed=True,
                failure_reason=(
                    f'pair_ratio={pair_ratio:.2f} < min_pair_ratio='
                    f'{self.cfg.min_pair_ratio} — tracking effectively lost'
                ),
                metrics={**report, 'pair_ratio': pair_ratio},
                artifacts={
                    'runner_log': str(log_path),
                    'validation_json': str(out_dir / 'validation.json'),
                },
            )

        # Score = drift_ratio + small penalty for low pair coverage.
        missing_poses = max(0, n_gt - n_pairs)
        score = drift_ratio + missing_poses * self.cfg.tracking_loss_penalty

        return EvaluationResult(
            score=float(score),
            failed=False,
            metrics={
                'ate_rmse_m': ate_rmse,
                'ate_mean_m': report.get('ate_mean_m'),
                'ate_max_m': report.get('ate_max_m'),
                'final_drift_m': report.get('final_drift_m'),
                'drift_ratio': drift_ratio,
                'pair_ratio': pair_ratio,
                'n_pairs': n_pairs,
                'n_gt_poses': n_gt,
                'trajectory_length_m': report.get('trajectory_length_m'),
            },
            artifacts={
                'runner_log': str(log_path),
                'validation_json': str(out_dir / 'validation.json'),
                'bag': str(out_dir / 'bag'),
                'rtabmap_db': str(out_dir / 'rtabmap.db'),
            },
        )


def _params_to_rtabmap_args(params: Dict[str, Any]) -> str:
    """Convert {Icp/VoxelSize: 0.05, ...} -> '--Icp/VoxelSize 0.05 ...'."""
    parts: List[str] = []
    for k, v in params.items():
        if isinstance(v, bool):
            v = 'true' if v else 'false'
        parts.append(f'--{k}')
        parts.append(str(v))
    return ' '.join(parts)
