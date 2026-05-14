"""Optuna-based optimizer for RTAB-Map parameters.

Each Optuna trial:
  1. Samples a parameter set from ``SEARCH_SPACE``.
  2. Runs ``trial_runner.run_trial`` on every supplied bag.
  3. Aggregates per-bag drift into a scalar score (mean drift, with a
     ``--failure-penalty-m`` contribution from any bag that didn't produce a
     usable trajectory).

The objective is minimization (drift = bad). The study is persisted as a
SQLite database under ``<output_root>/optuna.db`` so runs can be resumed.

For dev/CI, ``--objective synthetic`` swaps in a stub that scores the
suggested params against a hidden target and never spawns a real RTAB-Map
run. This lets us verify the optimizer plumbing without needing a bag.
"""

from __future__ import annotations

import argparse
import json
import queue
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional

import optuna

from .trial_runner import (
    EnvConfig,
    cleanup_orphan_trials,
    install_shutdown_handler,
    run_trial,
)


# ---------------------------------------------------------------------------
# Metric registry
# ---------------------------------------------------------------------------
# Each metric has a name, an optimization direction, how to pull its value out
# of a per-bag ``TrajectoryStats`` dict, and a fallback to contribute when a
# bag run failed. Aggregation across bags is always a mean.
#
# Add new metrics here. The optimizer will accept any registered name via
# ``--metric``. When multiple ``--metric`` flags are given, Optuna switches to
# multi-objective (Pareto) optimization with NSGA-II.
@dataclass(frozen=True)
class MetricSpec:
    name: str
    direction: str              # 'minimize' or 'maximize'
    extract: Callable[[dict], float]
    fail_value: float           # contributed when a bag run failed
    aggregator: str = 'median'  # 'median' | 'max' | 'min' | 'mean' | 'q75' | 'q90'


def _quantile(values: list[float], q: float) -> float:
    """Linear-interpolation quantile (like numpy.quantile with method='linear').

    Used for q75/q90 aggregators when worst-bag (max) is too sensitive to a
    single failing bag, but median is too forgiving of "2 of 10 bags failed."
    q75 = "75% of bags are at or below this drift" — moves with the worst
    quartile, not the single worst bag.
    """
    if not values:
        return 0.0
    if len(values) == 1:
        return float(values[0])
    s = sorted(values)
    idx = q * (len(s) - 1)
    lo = int(idx)
    hi = min(lo + 1, len(s) - 1)
    frac = idx - lo
    return s[lo] * (1.0 - frac) + s[hi] * frac


METRICS: dict[str, MetricSpec] = {
    # Absolute start-to-end pose drift. The original metric. Sensitive to
    # absolute scale and uninformative when the trajectory is short.
    'drift_m': MetricSpec(
        'drift_m', 'minimize',
        extract=lambda s: float(s['drift_m']),
        fail_value=5.0,
    ),
    # Drift normalized by path length (median across bags — rewards "typical
    # bag is OK"). Less sensitive to a single failed bag than mean.
    'drift_per_path': MetricSpec(
        'drift_per_path', 'minimize',
        extract=lambda s: float(s.get('drift_per_path') or 1.0),
        fail_value=1.0,
    ),
    # Worst-bag drift_per_path (max across bags). The "did we lose tracking
    # on ANY bag?" metric. If any bag shows catastrophic drift, this captures
    # it; median would hide a single-bag failure as long as 5+ others were OK.
    # Use this as the primary optimization target when tracking robustness
    # matters more than typical-case quality.
    'max_drift_per_path': MetricSpec(
        'max_drift_per_path', 'minimize',
        extract=lambda s: float(s.get('drift_per_path') or 1.0),
        fail_value=1.0,
        aggregator='max',
    ),
    # Worst-bag absolute drift (max across bags). For the same robustness
    # intent but in raw meters — easier to interpret as "no bag drifts more
    # than X meters."
    'max_drift_m': MetricSpec(
        'max_drift_m', 'minimize',
        extract=lambda s: float(s['drift_m']),
        fail_value=5.0,
        aggregator='max',
    ),
    # 75th-percentile drift across bags. The middle ground between median
    # (rewards "typical bag is OK", ignores 1-2 failures) and max (one bad
    # bag dominates). Useful when there's structural variance between bags
    # and a single hard-to-handle bag shouldn't dictate the entire score.
    'q75_drift_per_path': MetricSpec(
        'q75_drift_per_path', 'minimize',
        extract=lambda s: float(s.get('drift_per_path') or 1.0),
        fail_value=1.0,
        aggregator='q75',
    ),
    # 90th-percentile drift — closer to max, but still allows the single
    # worst bag to be an outlier rather than the optimization target.
    'q90_drift_per_path': MetricSpec(
        'q90_drift_per_path', 'minimize',
        extract=lambda s: float(s.get('drift_per_path') or 1.0),
        fail_value=1.0,
        aggregator='q90',
    ),
    # ICP odometry health: mean correspondence ratio over all scan registrations.
    # Higher = scans align cleanly. Ghosting almost always has low values here.
    'mean_icp_ratio': MetricSpec(
        'mean_icp_ratio', 'maximize',
        extract=lambda s: float(s.get('mean_icp_ratio') or 0.0),
        fail_value=0.0,
    ),
    # Number of accepted loop closures. A trajectory that revisits regions
    # without firing loop closures is locally consistent but globally drifting
    # — that's the textbook ghosting setup. More loops = more correction.
    'loop_closure_count': MetricSpec(
        'loop_closure_count', 'maximize',
        extract=lambda s: float(s.get('loop_closure_count') or 0),
        fail_value=0.0,
    ),
}


# ROS_DOMAIN_IDs reserved by the production system; the tuner must never use
# these for parallel workers because doing so would inject test traffic into
# the live robot's DDS network. 96 is the Rove robot's live domain.
RESERVED_DOMAIN_IDS: frozenset[int] = frozenset({0, 96})


def build_domain_pool(n_workers: int, exclude: frozenset[int] = RESERVED_DOMAIN_IDS) -> queue.Queue:
    """Return a Queue containing ``n_workers`` unique ROS_DOMAIN_IDs, none of
    which are in ``exclude``. Domains are pulled from [1, 99] (the safe range
    without DDS multicast restrictions).
    """
    pool: queue.Queue = queue.Queue()
    candidates = [d for d in range(1, 100) if d not in exclude]
    if n_workers > len(candidates):
        raise ValueError(
            f'n_workers={n_workers} exceeds available domain IDs ({len(candidates)} after exclusions)'
        )
    for d in candidates[:n_workers]:
        pool.put(d)
    return pool


# ---------------------------------------------------------------------------
# Search space
# ---------------------------------------------------------------------------
# (kind, *args) per key. Pick params with real, well-understood impact on
# odometry quality; leave environmental knobs (Grid/*, frame ids, etc.) fixed.
#
#   ('float', low, high, log_scale)
#   ('int',   low, high)
#   ('cat',   [choices...])
SEARCH_SPACE: dict[str, tuple] = {
    # ICP shared. voxel_size and max_correspondence_distance ranges narrowed
    # after observing that large-voxel optima (~0.3m) cause visible ghosting
    # in the assembled map even when they minimize start-to-end drift.
    # 10cm voxel is the practical upper bound for a clean outdoor map; the
    # 10x voxel/correspondence ratio is preserved.
    'icp_voxel_size':                  ('float', 0.01, 0.1, True),
    'icp_max_correspondence_distance': ('float', 0.05, 1.0, True),
    'icp_iterations':                  ('int', 5, 50),
    'icp_outlier_ratio':               ('float', 0.1, 0.9, False),
    'icp_max_translation':             ('float', 0.1, 2.0, False),
    'icp_point_to_plane_k':            ('int', 5, 50),
    # icp_strategy '2' = PCL Generalized ICP. Removed from the search space
    # because rtabmap_odom segfaults on it (exit code -11) for many real
    # point-cloud distributions. Reproduced consistently across trials in
    # capra_full_v1 (470+) and matches a known PCL GICP eigendecomposition
    # crash. '0' (PCL point-to-point) and '1' (libpointmatcher) are stable.
    'icp_strategy':                    ('cat', ['0', '1']),

    # ICP odometry. Ranges narrowed after observing 4/5 trials with the wider
    # bounds reject every scan ("no odometry poses"). The most sensitive knob
    # is icp_odom_correspondence_ratio: at 0.4+ the scan-vs-localmap overlap
    # check rejects everything. odomf2m_scan_subtract_radius >0.2 strips too
    # many points from the local map to find correspondences.
    'odom_scan_keyframe_thr':          ('float', 0.1, 0.9, False),
    'odomf2m_scan_max_size':           ('int', 5000, 50000),
    'odomf2m_scan_subtract_radius':    ('float', 0.005, 0.2, True),
    'icp_odom_correspondence_ratio':   ('float', 0.01, 0.2, False),

    # RTAB-Map SLAM / memory
    'rgbd_linear_update':              ('float', 0.05, 0.5, False),
    'rgbd_angular_update':             ('float', 0.05, 0.5, False),
    # Lower STM (Short-Term Memory) so nodes graduate to Working Memory
    # quickly — only WM nodes are eligible candidates for proximity loop
    # closure. With STM=30 and short trajectories (<30 keyframes), no node is
    # ever eligible, so loop_closure_count is stuck at zero.
    'mem_stm_size':                    ('int', 2, 15),
    'icp_map_correspondence_ratio':    ('float', 0.05, 0.5, False),
    # Number of candidate neighbors checked per step for proximity (space-based)
    # loop closure. Higher = more aggressive loop closure attempts (more CPU).
    # The launch default of 1 is conservative and likely too tight for short
    # trajectories where revisits are uncommon.
    'rgbd_proximity_path_max_neighbors': ('int', 1, 10),
}


def suggest_params(
    trial: optuna.Trial,
    search_space: dict[str, tuple] = SEARCH_SPACE,
) -> dict[str, object]:
    """Translate ``search_space`` entries into Optuna ``suggest_*`` calls."""
    params: dict[str, object] = {}
    for key, spec in search_space.items():
        kind = spec[0]
        if kind == 'float':
            _, low, high, log = spec
            params[key] = trial.suggest_float(key, low, high, log=log)
        elif kind == 'int':
            _, low, high = spec
            params[key] = trial.suggest_int(key, low, high)
        elif kind == 'cat':
            _, choices = spec
            params[key] = trial.suggest_categorical(key, choices)
        else:
            raise ValueError(f'Unknown search-space kind {kind!r} for key {key!r}')
    return params


# ---------------------------------------------------------------------------
# Real objective
# ---------------------------------------------------------------------------
def _aggregate_metric(metric: MetricSpec, bag_results) -> tuple[float, list[float]]:
    """Aggregate per-bag values into a single score per the metric's
    ``aggregator`` field. Returns (aggregated_value, per_bag_values).

    Median rewards "typical bag is OK"; max rewards "every bag is OK" (which
    matters more when tracking-loss on any single bag is the failure mode
    we care about). Failed bags contribute ``fail_value`` to the input list.
    """
    values: list[float] = []
    for run in bag_results:
        metrics = run.metrics or {}
        stats = metrics.get('stats') if metrics.get('success') else None
        if run.success and stats is not None:
            try:
                values.append(metric.extract(stats))
            except (KeyError, TypeError):
                values.append(metric.fail_value)
        else:
            values.append(metric.fail_value)

    if not values:
        return metric.fail_value, values
    if metric.aggregator == 'median':
        return statistics.median(values), values
    if metric.aggregator == 'max':
        return max(values), values
    if metric.aggregator == 'min':
        return min(values), values
    if metric.aggregator == 'mean':
        return sum(values) / len(values), values
    if metric.aggregator == 'q75':
        return _quantile(values, 0.75), values
    if metric.aggregator == 'q90':
        return _quantile(values, 0.90), values
    raise ValueError(f'unknown aggregator {metric.aggregator!r} for metric {metric.name!r}')


def make_real_objective(
    bags: list[Path],
    env: EnvConfig,
    output_root: Path,
    *,
    warmup_s: float,
    drain_s: float,
    shutdown_timeout_s: float,
    max_bag_duration_s: Optional[float],
    metrics: list[str],
    search_space: dict[str, tuple] = SEARCH_SPACE,
    domain_pool: Optional[queue.Queue] = None,
):
    """Build an Optuna objective that runs ``run_trial`` and returns one value
    per requested metric. When a single metric is requested, returns a scalar
    (single-objective). When multiple are requested, returns a tuple
    (multi-objective — Optuna treats this as Pareto optimization).

    All available metrics are always written to ``trial.user_attrs`` for
    post-hoc analysis, regardless of which were used to drive the search.
    """
    specs = [METRICS[m] for m in metrics]

    def objective(trial: optuna.Trial):
        overrides = suggest_params(trial, search_space)
        trial_id = f'trial_{trial.number:04d}'

        domain_id = None
        if domain_pool is not None:
            domain_id = domain_pool.get()

        try:
            result = run_trial(
                trial_id=trial_id,
                overrides=overrides,
                bags=bags,
                output_root=output_root,
                env=env,
                warmup_s=warmup_s,
                drain_s=drain_s,
                shutdown_timeout_s=shutdown_timeout_s,
                max_bag_duration_s=max_bag_duration_s,
                ros_domain_id=domain_id,
            )
        finally:
            if domain_id is not None:
                domain_pool.put(domain_id)

        # Compute and record every registered metric (not just the ones we
        # optimize on) so post-hoc analysis can rank by any of them.
        all_values: dict[str, float] = {}
        per_bag_breakdown: dict[str, list[float]] = {}
        for name, spec in METRICS.items():
            mean, per_bag = _aggregate_metric(spec, result.runs)
            all_values[name] = mean
            per_bag_breakdown[name] = per_bag

        # Renamed from 'all_metric_means' (which it never was, even before the
        # switch from mean→median) to be aggregation-agnostic. Old studies'
        # user_attrs still carry the old key; analysis code below handles both.
        trial.set_user_attr('all_metric_aggregates', all_values)
        trial.set_user_attr('per_bag', per_bag_breakdown)
        trial.set_user_attr('n_bags_successful',
                            sum(1 for r in result.runs if r.success))
        if domain_id is not None:
            trial.set_user_attr('ros_domain_id', domain_id)

        objective_values = tuple(all_values[m] for m in metrics)
        return objective_values[0] if len(specs) == 1 else objective_values

    return objective


# ---------------------------------------------------------------------------
# Synthetic objective (for testing the optimizer machinery without bags)
# ---------------------------------------------------------------------------
# Hidden "best" values. Distinct from the launch defaults so TPE has a real
# valley to descend into rather than already-good starting points.
_SYNTHETIC_TARGET: dict[str, object] = {
    'icp_voxel_size': 0.07,
    'icp_max_correspondence_distance': 0.7,
    'icp_iterations': 20,
    'icp_outlier_ratio': 0.5,
    'icp_max_translation': 0.8,
    'icp_point_to_plane_k': 15,
    'icp_strategy': '1',
    'odom_scan_keyframe_thr': 0.45,
    'odomf2m_scan_max_size': 20000,
    'odomf2m_scan_subtract_radius': 0.05,
    'icp_odom_correspondence_ratio': 0.1,
    'rgbd_linear_update': 0.2,
    'rgbd_angular_update': 0.2,
    'mem_stm_size': 50,
    'icp_map_correspondence_ratio': 0.2,
}


def make_synthetic_objective(
    search_space: dict[str, tuple] = SEARCH_SPACE,
) -> Callable[[optuna.Trial], float]:
    def objective(trial: optuna.Trial) -> float:
        params = suggest_params(trial, search_space)
        score = 0.0
        for key, value in params.items():
            target = _SYNTHETIC_TARGET.get(key)
            if target is None:
                continue
            if isinstance(value, str):
                score += 0.0 if value == str(target) else 1.0
            else:
                tnum = float(target)
                vnum = float(value)
                score += ((vnum - tnum) / max(abs(tnum), 1e-6)) ** 2
        return score
    return objective


# ---------------------------------------------------------------------------
# Study runner
# ---------------------------------------------------------------------------
def run_study(
    objective,
    *,
    study_name: str,
    storage: str,
    n_trials: int,
    metrics: list[str],
    seed: int = 42,
    callbacks: Optional[list] = None,
    n_jobs: int = 1,
) -> optuna.Study:
    directions = [METRICS[m].direction for m in metrics]
    if len(metrics) == 1:
        sampler = optuna.samplers.TPESampler(seed=seed)
        study = optuna.create_study(
            direction=directions[0],
            sampler=sampler,
            study_name=study_name,
            storage=storage,
            load_if_exists=True,
        )
    else:
        # Multi-objective TPE (replaces NSGAIISampler after observing NSGA-II
        # plateau on capra_full_v1 — 100+ trials with no new Pareto contributions).
        # TPESampler in Optuna 4.x handles multi-objective directly when the
        # study has multiple `directions`. multivariate=True models parameter
        # correlations (RTAB-Map's voxel size and correspondence distance are
        # tightly coupled — independent sampling wastes most candidates).
        sampler = optuna.samplers.TPESampler(seed=seed, multivariate=True)
        study = optuna.create_study(
            directions=directions,
            sampler=sampler,
            study_name=study_name,
            storage=storage,
            load_if_exists=True,
        )

    # Self-heal: any RUNNING trials from a previous force-kill are stale.
    # Mark them FAIL before resuming so TPE doesn't try to read their
    # nonexistent results and crash with "Cannot tell a FAIL trial."
    stale = [
        t for t in study.get_trials(deepcopy=False, states=(optuna.trial.TrialState.RUNNING,))
    ]
    for t in stale:
        try:
            study._storage.set_trial_state_values(t._trial_id, state=optuna.trial.TrialState.FAIL)
        except Exception:
            pass
    if stale:
        print(f'[startup] marked {len(stale)} stale RUNNING trials as FAIL', flush=True)

    _resilient_optimize(study, objective, n_trials=n_trials,
                        callbacks=callbacks or [], n_jobs=n_jobs)
    return study


def _resilient_optimize(
    study: optuna.Study,
    objective,
    *,
    n_trials: int,
    callbacks: list,
    n_jobs: int,
    max_consecutive_crashes: int = 5,
) -> None:
    """Wrap ``study.optimize`` so that an Optuna-level crash (e.g. the
    ``Cannot tell a FAIL trial`` race triggered when external SQL or another
    process mutates a RUNNING trial mid-flight) doesn't kill the whole run.

    Each iteration:
      1. Call ``study.optimize(n_trials=remaining)``.
      2. On internal Optuna error (UpdateFinishedTrialError / its secondary
         ValueError from ``_tell``), mark any leftover RUNNING trials as FAIL
         so the next iteration starts from a clean slate, then retry.
      3. Count completed trials each iteration to know progress; if multiple
         iterations make ZERO progress in a row, give up — the failure is
         persistent, not just a race.
    """
    target_completed = (
        len([t for t in study.trials if t.state == optuna.trial.TrialState.COMPLETE])
        + n_trials
    )
    consecutive_no_progress = 0

    while True:
        completed_now = len([t for t in study.trials if t.state == optuna.trial.TrialState.COMPLETE])
        remaining = target_completed - completed_now
        if remaining <= 0:
            break

        try:
            study.optimize(objective, n_trials=remaining, callbacks=callbacks, n_jobs=n_jobs)
            break  # natural completion
        except (ValueError, optuna.exceptions.UpdateFinishedTrialError) as exc:
            print(f'[run_study] {type(exc).__name__}: {exc}', flush=True)
            print('[run_study] cleaning up orphan RUNNING trials and resuming...', flush=True)
            for t in study.get_trials(
                states=(optuna.trial.TrialState.RUNNING,), deepcopy=False
            ):
                try:
                    study._storage.set_trial_state_values(
                        t._trial_id, state=optuna.trial.TrialState.FAIL
                    )
                except Exception:
                    pass

            done_after = len(
                [t for t in study.trials if t.state == optuna.trial.TrialState.COMPLETE]
            )
            progressed = done_after > completed_now
            consecutive_no_progress = 0 if progressed else consecutive_no_progress + 1
            if consecutive_no_progress >= max_consecutive_crashes:
                print(
                    f'[run_study] aborting: {max_consecutive_crashes} crashes in a row '
                    f'without any trial completing — failure looks persistent, not a race.',
                    flush=True,
                )
                raise


def write_study_summary(study: optuna.Study, output_path: Path, metrics: list[str]) -> None:
    # Optuna raises on FrozenTrial.value during multi-objective studies (only
    # .values is valid then), and vice-versa. Pick which attribute to read
    # based on the configured metric count.
    multi = len(metrics) > 1

    def trial_obj_values(t: optuna.trial.FrozenTrial) -> Optional[list[float]]:
        if multi:
            return list(t.values) if t.values is not None else None
        return [t.value] if t.value is not None else None

    finished = [
        t for t in study.trials
        if t.state == optuna.trial.TrialState.COMPLETE and trial_obj_values(t) is not None
    ]

    def trial_payload(t: optuna.trial.FrozenTrial) -> dict:
        attrs = dict(t.user_attrs)
        vals = trial_obj_values(t) or []
        # Read either the new or legacy key (older trials predate the rename).
        all_agg = attrs.get('all_metric_aggregates') or attrs.get('all_metric_means', {})
        return {
            'number': t.number,
            'objective_values': {m: v for m, v in zip(metrics, vals)},
            'all_metric_aggregates': all_agg,
            'params': t.params,
            'user_attrs': attrs,
        }

    # For each requested metric, list the top 5 trials by that metric.
    top_per_metric: dict[str, list[dict]] = {}
    for i, m in enumerate(metrics):
        reverse = METRICS[m].direction == 'maximize'
        def keyfn(t, idx=i):
            vals = trial_obj_values(t)
            return vals[idx] if vals is not None else float('inf')
        sorted_trials = sorted(finished, key=keyfn, reverse=reverse)
        top_per_metric[m] = [trial_payload(t) for t in sorted_trials[:5]]

    summary = {
        'study_name': study.study_name,
        'metrics': metrics,
        'directions': [METRICS[m].direction for m in metrics],
        'n_trials_total': len(study.trials),
        'n_trials_finished': len(finished),
        'top_5_per_metric': top_per_metric,
    }

    if multi:
        # Multi-objective: the Pareto front is the set of non-dominated trials.
        pareto = study.best_trials
        summary['pareto_front'] = [trial_payload(t) for t in pareto]
    else:
        # Single-objective: the unique best trial is unambiguous.
        summary['best_trial'] = top_per_metric[metrics[0]][0] if finished else None

    output_path.write_text(json.dumps(summary, indent=2, sort_keys=True))


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def _parse_bool(s: str) -> bool:
    return s.strip().lower() in ('true', '1', 'yes', 'y')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        '--objective', choices=['real', 'synthetic'], default='real',
        help='real = run RTAB-Map per trial; synthetic = stub for testing the optimizer.',
    )
    parser.add_argument(
        '--bag', '-b', type=Path, action='append', default=[], dest='bags',
        help='Path to a rosbag. Repeat for multiple bags. Required for --objective real.',
    )
    parser.add_argument(
        '--output-root', '-o', type=Path,
        help='Root directory for the study DB and per-trial outputs. '
             'Required unless --list-search-space.',
    )
    parser.add_argument('--study-name', default='rtabmap_tuning')
    parser.add_argument('--n-trials', type=int, default=30)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument(
        '--metric', action='append', default=[], dest='metrics',
        choices=sorted(METRICS),
        help='Metric to optimize. Repeat for multi-objective (Pareto) optimization '
             'with NSGA-II. Default: drift_per_path. All metrics are always tracked '
             'in trial.user_attrs regardless of which drive the search.',
    )
    parser.add_argument(
        '--failure-penalty-m', type=float, default=100.0,
        help='(Legacy; ignored — per-metric fail_value in METRICS is now used.)',
    )
    parser.add_argument(
        '--list-search-space', action='store_true',
        help='Print the current search space and exit.',
    )
    parser.add_argument(
        '--list-metrics', action='store_true',
        help='Print all registered metrics and their directions, then exit.',
    )

    # Pass-through to the trial runner.
    parser.add_argument('--lidar-topic', default='/livox/lidar')
    parser.add_argument('--imu-topic', default='/imu/data')
    parser.add_argument('--frame-id', default='base_link')
    parser.add_argument('--fixed-frame-id', default='')
    parser.add_argument('--expected-update-rate', type=float, default=15.0)
    parser.add_argument('--qos', type=int, default=1)
    parser.add_argument('--deskewing', type=_parse_bool, default=True)
    parser.add_argument('--warmup-s', type=float, default=5.0)
    parser.add_argument('--drain-s', type=float, default=3.0)
    parser.add_argument('--shutdown-timeout-s', type=float, default=30.0)
    parser.add_argument('--max-bag-duration-s', type=float, default=None)
    parser.add_argument(
        '--bag-play-arg', action='append', default=[],
        help='Extra arg passed to `ros2 bag play` (after `--clock`). Repeat for multiple. '
             "Use `=` binding (e.g. `--bag-play-arg=--topics`) when the value starts with `--`.",
    )
    parser.add_argument(
        '--n-jobs', type=int, default=1,
        help='Number of trials to run in parallel. Each concurrent worker gets a unique '
             'ROS_DOMAIN_ID so DDS topics stay isolated. Reserved domains (0 and 96) are '
             'always skipped; 96 is the live Rove robot.',
    )

    args = parser.parse_args()

    if args.list_search_space:
        for key, spec in SEARCH_SPACE.items():
            print(f'{key}: {spec}')
        return 0

    if args.list_metrics:
        for name, spec in METRICS.items():
            print(f'{name:24s}  direction={spec.direction:<8s}  fail_value={spec.fail_value}')
        return 0

    if not args.metrics:
        args.metrics = ['drift_per_path']

    if args.output_root is None:
        parser.error('--output-root is required unless --list-search-space is given')

    args.output_root.mkdir(parents=True, exist_ok=True)

    # Install SIGINT/SIGTERM handler so force-kill terminates all subprocess
    # groups instead of orphaning rtabmap / ros2 bag children.
    install_shutdown_handler()

    # Self-heal from any previous force-kill: clean up trial directories
    # that lack a trial.json marker (incomplete trials), and mark any
    # RUNNING rows in the Optuna DB as FAIL. Done before we touch the study
    # so TPE doesn't get confused by half-written priors.
    cleanup_stats = cleanup_orphan_trials(args.output_root)
    if cleanup_stats['removed']:
        print(f'[startup] cleaned up {cleanup_stats["removed"]} incomplete trial dirs '
              f'(of {cleanup_stats["inspected"]} inspected)', flush=True)

    if args.objective == 'real':
        if not args.bags:
            parser.error('--bag is required for --objective real')
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
        domain_pool = build_domain_pool(args.n_jobs) if args.n_jobs > 1 else None
        objective = make_real_objective(
            bags=args.bags,
            env=env,
            output_root=args.output_root,
            warmup_s=args.warmup_s,
            drain_s=args.drain_s,
            shutdown_timeout_s=args.shutdown_timeout_s,
            max_bag_duration_s=args.max_bag_duration_s,
            metrics=args.metrics,
            domain_pool=domain_pool,
        )
    else:
        objective = make_synthetic_objective()

    storage = f'sqlite:///{args.output_root.resolve()}/optuna.db'

    metrics = args.metrics
    multi = len(metrics) > 1

    def progress(study: optuna.Study, trial: optuna.trial.FrozenTrial) -> None:
        if multi:
            vals = trial.values
            val_s = 'failed' if vals is None else ', '.join(f'{v:.4f}' for v in vals)
            try:
                n_pareto = len(study.best_trials)
            except Exception:
                n_pareto = 0
            print(f'[trial {trial.number:04d}] values=({val_s})  pareto_size={n_pareto}', flush=True)
        else:
            try:
                best_val = study.best_value
            except ValueError:
                best_val = float('inf')
            val_s = 'failed' if trial.value is None else f'{trial.value:.4f}'
            print(f'[trial {trial.number:04d}] {metrics[0]}={val_s}  best_so_far={best_val:.4f}', flush=True)

    optuna.logging.set_verbosity(optuna.logging.WARNING)
    study = run_study(
        objective=objective,
        study_name=args.study_name,
        storage=storage,
        n_trials=args.n_trials,
        metrics=metrics,
        seed=args.seed,
        callbacks=[progress],
        n_jobs=args.n_jobs,
    )

    summary_path = args.output_root / 'study_summary.json'
    write_study_summary(study, summary_path, metrics)

    print()
    if multi:
        try:
            pareto = study.best_trials
        except Exception:
            pareto = []
        print(f'Pareto front: {len(pareto)} non-dominated trials')
        print(f'Top trial per metric:')
        for i, m in enumerate(metrics):
            direction = METRICS[m].direction
            reverse = direction == 'maximize'
            try:
                best = sorted(
                    [t for t in study.trials if t.values is not None],
                    key=lambda t, i=i: t.values[i],
                    reverse=reverse,
                )[0]
                vstr = ', '.join(f'{mm}={v:.4f}' for mm, v in zip(metrics, best.values))
                print(f'  {m} ({direction}): trial #{best.number}  ({vstr})')
            except IndexError:
                print(f'  {m}: no finished trials')
    else:
        try:
            best = study.best_trial
            print(f'Best trial: #{best.number}  {metrics[0]}={best.value:.4f}')
            print('Best params:')
            for k, v in sorted(best.params.items()):
                print(f'  {k} = {v}')
        except ValueError:
            print('No successful trials.')
    print(f'Summary: {summary_path}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
