"""RealEvaluator — run candidate params against a real bag, compare to ref db.

For phase-2 validation: the top-K candidates from phase-1 (sim) get re-run on
real bags and graded against a reference (cleaned-up) RTAB-Map database.

The score is a weighted combination of:
    correspondence_ratio (higher better; flipped to lower-better in the score)
    tracking_loss_ratio  (lower better)
both produced by reference_compare.compare().

Each bag is evaluated independently. When several bags are configured, the
aggregator (default median) reduces per-bag scores to one trial score.

Note: this evaluator launches RTAB-Map via the *project's existing* tuner
launch template (rove_rtabmap_tuner.templates.lidar3d_tunable) rather than
the sim launch — we want the real-data inference path here.
"""

from __future__ import annotations

import json
import os
import subprocess
import time
import statistics
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

from rove_tuning_orchestrator.evaluators.base import EvaluationResult, Evaluator
from rove_tuning_orchestrator.reference_compare import compare as ref_compare


@dataclass
class RealEvaluatorConfig:
    # Paths.
    bags: List[Path] = field(default_factory=list)
    reference_db: Path = Path('~/bags/reference.db').expanduser()
    # Topic plumbing — matches the tuner template defaults.
    lidar_topic: str = '/livox/lidar'
    imu_topic: str = '/livox/imu'
    frame_id: str = 'livox_frame'
    # Scoring weights.
    corr_weight: float = 1.0
    loss_weight: float = 2.0   # tracking loss is worse than poor correspondence
    tau_m: float = 0.5
    # Bag-aggregation: 'median' | 'mean' | 'max'.
    aggregator: str = 'median'
    # Per-bag run constraints.
    timeout_s: float = 600.0
    max_bag_duration_s: Optional[float] = 180.0
    warmup_s: float = 8.0
    drain_s: float = 3.0
    domain_id: int = 123
    expected_update_rate: float = 50.0
    # If RTAB-Map produces no db, the trial is failed.
    # If correspondence_ratio < this threshold for any bag, treat as failed.
    min_correspondence_ratio: float = 0.05
    # Minimum number of candidate nodes for a bag to be "fully credited" —
    # below this, correspondence_ratio is linearly attenuated so trivially-
    # matched few-node runs don't game the score. Defeats the artifact where
    # a candidate that produces 7 nodes can trivially match all of them.
    min_candidate_nodes: int = 20


class RealEvaluator(Evaluator):
    name = 'real'

    def __init__(self, cfg: RealEvaluatorConfig):
        if not cfg.bags:
            raise ValueError('RealEvaluator needs at least one bag in cfg.bags')
        if not cfg.reference_db.exists():
            raise ValueError(
                f'Reference db not found at {cfg.reference_db}. '
                f'Set RealEvaluatorConfig.reference_db to a cleaned-up rtabmap.db.'
            )
        self.cfg = cfg

    def evaluate(
        self,
        params: Dict[str, Any],
        trial_id: str,
        out_dir: Path,
    ) -> EvaluationResult:
        out_dir.mkdir(parents=True, exist_ok=True)

        per_bag_results: List[Dict[str, Any]] = []
        per_bag_scores: List[float] = []
        warnings: List[str] = []
        failed_any = False
        first_failure_reason = ''

        for bag in self.cfg.bags:
            bag_out = out_dir / bag.name
            bag_out.mkdir(parents=True, exist_ok=True)
            try:
                bag_score, bag_metrics, bag_warnings = self._evaluate_one_bag(
                    params, bag, bag_out,
                )
            except _RealEvalFailure as e:
                bag_metrics = {'failed': True, 'reason': str(e)}
                bag_warnings = []
                bag_score = float('inf')
                failed_any = True
                if not first_failure_reason:
                    first_failure_reason = f'{bag.name}: {e}'
            per_bag_results.append({'bag': bag.name, **bag_metrics})
            per_bag_scores.append(bag_score)
            warnings.extend(bag_warnings)

        agg = _aggregate(per_bag_scores, self.cfg.aggregator)
        result = EvaluationResult(
            score=float(agg),
            failed=failed_any and agg == float('inf'),
            failure_reason=first_failure_reason,
            metrics={
                'aggregated_score': agg,
                'aggregator': self.cfg.aggregator,
                'per_bag': per_bag_results,
            },
            artifacts={'out_dir': str(out_dir)},
            warnings=warnings,
        )
        (out_dir / 'real_eval.json').write_text(
            json.dumps({
                'score': float(agg),
                'per_bag': per_bag_results,
                'warnings': warnings,
            }, indent=2)
        )
        return result

    def _evaluate_one_bag(
        self,
        params: Dict[str, Any],
        bag: Path,
        bag_out: Path,
    ):
        cfg = self.cfg
        db_path = bag_out / 'rtabmap.db'
        if db_path.exists():
            db_path.unlink()

        env = os.environ.copy()
        env['ROS_DOMAIN_ID'] = str(cfg.domain_id)

        # Build the rtabmap_launch command. Same pattern as the tuner's
        # trial_runner uses, but bypassing the trial_runner's Jinja step —
        # we just inline the args.
        param_args: List[str] = []
        for k, v in params.items():
            if isinstance(v, bool):
                v = 'true' if v else 'false'
            param_args.extend([f'--{k}', str(v)])

        # NOTE: we deliberately avoid rtabmap_launch.launch.py because its
        # subscribe_rgbd:=false arg does NOT prevent the rtabmap node from
        # exact-sync subscribing to camera/depth/rgb topics — same gotcha
        # the tuner has. Instead we use our own launcher
        # (rtabmap_lidar3d.launch.py) which directly invokes the two nodes
        # with subscribe_rgbd=False on the params, which works.
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
        rtabmap_log = bag_out / 'rtabmap.log'
        rtabmap_proc = subprocess.Popen(
            rtabmap_cmd, env=env,
            stdout=open(rtabmap_log, 'wb'), stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )

        time.sleep(cfg.warmup_s)

        # Play the bag.
        # NOTE: ROS 2 Humble's `ros2 bag play` does NOT support
        # --playback-duration (added in Jazzy/Iron). We instead launch with
        # Popen and kill after max_bag_duration_s ourselves.
        bag_log = bag_out / 'bag_play.log'
        bag_play_cmd = [
            'ros2', 'bag', 'play', str(bag),
            '--clock',
            '--topics', cfg.lidar_topic, cfg.imu_topic, '/tf', '/tf_static',
        ]
        bag_proc = subprocess.Popen(
            bag_play_cmd, env=env,
            stdout=open(bag_log, 'wb'), stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
        deadline_s = cfg.max_bag_duration_s or cfg.timeout_s
        try:
            bag_proc.wait(timeout=deadline_s)
        except subprocess.TimeoutExpired:
            # Expected when max_bag_duration_s caps the run; not an error.
            _kill_group(bag_proc, timeout=10)
        except Exception:
            _kill_group(bag_proc, timeout=10)
            _kill_group(rtabmap_proc, timeout=10)
            raise

        time.sleep(cfg.drain_s)
        _kill_group(rtabmap_proc)

        if not db_path.exists():
            raise _RealEvalFailure('RTAB-Map produced no database file')

        cmp_result = ref_compare(db_path, cfg.reference_db, tau_m=cfg.tau_m)
        (bag_out / 'reference_compare.json').write_text(
            json.dumps(_asdict(cmp_result), indent=2)
        )

        if cmp_result.correspondence_ratio < cfg.min_correspondence_ratio:
            raise _RealEvalFailure(
                f'correspondence_ratio={cmp_result.correspondence_ratio:.3f} '
                f'< {cfg.min_correspondence_ratio} (tracking quality unacceptable)'
            )

        # Score: lower is better. Penalize low corr_ratio, high tracking loss.
        # Few-node attenuation: linearly down-weight corr_ratio when the
        # candidate produced fewer than `min_candidate_nodes` — a 7-node bag
        # that hits corr_ratio=1.0 trivially gets attenuated to 1.0 * 7/20.
        node_factor = min(1.0, cmp_result.n_candidate_nodes / float(cfg.min_candidate_nodes))
        effective_corr = cmp_result.correspondence_ratio * node_factor
        score = (
            cfg.corr_weight * (1.0 - effective_corr)
            + cfg.loss_weight * cmp_result.tracking_loss_ratio
        )
        return float(score), {
            'correspondence_ratio': cmp_result.correspondence_ratio,
            'effective_corr_ratio': effective_corr,
            'node_factor': node_factor,
            'tracking_loss_ratio': cmp_result.tracking_loss_ratio,
            'tracking_loss_events': cmp_result.tracking_loss_events,
            'n_candidate_nodes': cmp_result.n_candidate_nodes,
            'n_reference_nodes': cmp_result.n_reference_nodes,
            'mean_nn_distance_m': cmp_result.mean_nn_distance_m,
            'score': score,
            'db_path': str(db_path),
        }, list(cmp_result.warnings)


class _RealEvalFailure(Exception):
    pass


def _kill_group(proc: subprocess.Popen, timeout: float = 10.0) -> None:
    import signal
    if proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        try:
            proc.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except ProcessLookupError:
        pass


def _aggregate(scores: List[float], how: str) -> float:
    finite = [s for s in scores if s != float('inf')]
    if not finite:
        return float('inf')
    if how == 'mean':
        return statistics.mean(finite)
    if how == 'max':
        return max(finite)
    return statistics.median(finite)


def _asdict(obj) -> dict:
    from dataclasses import asdict as _da
    return _da(obj)
