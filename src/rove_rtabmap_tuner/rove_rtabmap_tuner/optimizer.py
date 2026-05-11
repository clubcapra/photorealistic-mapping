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
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional

import optuna

from .trial_runner import EnvConfig, run_trial


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
    # ICP shared
    'icp_voxel_size':                  ('float', 0.01, 0.5, True),
    'icp_max_correspondence_distance': ('float', 0.05, 5.0, True),
    'icp_iterations':                  ('int', 5, 50),
    'icp_outlier_ratio':               ('float', 0.1, 0.9, False),
    'icp_max_translation':             ('float', 0.1, 2.0, False),
    'icp_point_to_plane_k':            ('int', 5, 50),
    'icp_strategy':                    ('cat', ['0', '1', '2']),

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
    'mem_stm_size':                    ('int', 10, 100),
    'icp_map_correspondence_ratio':    ('float', 0.05, 0.5, False),
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
def make_real_objective(
    bags: list[Path],
    env: EnvConfig,
    output_root: Path,
    *,
    warmup_s: float,
    drain_s: float,
    shutdown_timeout_s: float,
    max_bag_duration_s: Optional[float],
    failure_penalty_m: float,
    search_space: dict[str, tuple] = SEARCH_SPACE,
    domain_pool: Optional[queue.Queue] = None,
) -> Callable[[optuna.Trial], float]:
    """Build an Optuna objective that runs ``run_trial`` and returns the
    mean per-bag drift, with ``failure_penalty_m`` contributed for any bag
    that didn't produce a usable trajectory.

    If ``domain_pool`` is provided, each invocation pulls one ROS_DOMAIN_ID
    from the pool, uses it for the trial's subprocesses, and returns it on
    exit. This is what makes ``n_jobs > 1`` safe — concurrent trials use
    distinct DDS domains so their topics don't bleed into each other.
    """
    def objective(trial: optuna.Trial) -> float:
        overrides = suggest_params(trial, search_space)
        trial_id = f'trial_{trial.number:04d}'

        domain_id = None
        if domain_pool is not None:
            domain_id = domain_pool.get()  # blocks if all workers busy

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

        per_bag_drifts: list[float] = []
        for run in result.runs:
            metrics = run.metrics or {}
            stats = metrics.get('stats') if metrics.get('success') else None
            if run.success and stats is not None:
                per_bag_drifts.append(float(stats['drift_m']))
            else:
                per_bag_drifts.append(failure_penalty_m)

        trial.set_user_attr('per_bag_drifts', per_bag_drifts)
        trial.set_user_attr('n_bags_successful',
                            sum(1 for r in result.runs if r.success))
        if domain_id is not None:
            trial.set_user_attr('ros_domain_id', domain_id)
        return sum(per_bag_drifts) / len(per_bag_drifts)
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
    objective: Callable[[optuna.Trial], float],
    *,
    study_name: str,
    storage: str,
    n_trials: int,
    seed: int = 42,
    callbacks: Optional[list] = None,
    n_jobs: int = 1,
) -> optuna.Study:
    sampler = optuna.samplers.TPESampler(seed=seed)
    study = optuna.create_study(
        direction='minimize',
        sampler=sampler,
        study_name=study_name,
        storage=storage,
        load_if_exists=True,
    )
    study.optimize(objective, n_trials=n_trials, callbacks=callbacks or [], n_jobs=n_jobs)
    return study


def write_study_summary(study: optuna.Study, output_path: Path) -> None:
    finished = [t for t in study.trials if t.value is not None]
    best = study.best_trial if finished else None
    top5 = sorted(finished, key=lambda t: t.value)[:5]

    summary = {
        'study_name': study.study_name,
        'direction': study.direction.name,
        'n_trials_total': len(study.trials),
        'n_trials_finished': len(finished),
        'best_trial': None if best is None else {
            'number': best.number,
            'value': best.value,
            'params': best.params,
            'user_attrs': dict(best.user_attrs),
        },
        'top_5': [
            {
                'number': t.number,
                'value': t.value,
                'params': t.params,
                'user_attrs': dict(t.user_attrs),
            }
            for t in top5
        ],
    }
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
        '--failure-penalty-m', type=float, default=100.0,
        help='Drift contribution for a failed bag (default: 100m).',
    )
    parser.add_argument(
        '--list-search-space', action='store_true',
        help='Print the current search space and exit.',
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

    if args.output_root is None:
        parser.error('--output-root is required unless --list-search-space is given')

    args.output_root.mkdir(parents=True, exist_ok=True)

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
            failure_penalty_m=args.failure_penalty_m,
            domain_pool=domain_pool,
        )
    else:
        objective = make_synthetic_objective()

    storage = f'sqlite:///{args.output_root.resolve()}/optuna.db'

    def progress(study: optuna.Study, trial: optuna.trial.FrozenTrial) -> None:
        try:
            best_val = study.best_value
        except ValueError:
            best_val = float('inf')
        val_s = 'failed' if trial.value is None else f'{trial.value:.4f}'
        # flush=True so progress shows up live when stdout is redirected to a
        # file (the default Python stdout buffering hides progress otherwise).
        print(f'[trial {trial.number:04d}] value={val_s}  best_so_far={best_val:.4f}', flush=True)

    optuna.logging.set_verbosity(optuna.logging.WARNING)
    study = run_study(
        objective=objective,
        study_name=args.study_name,
        storage=storage,
        n_trials=args.n_trials,
        seed=args.seed,
        callbacks=[progress],
        n_jobs=args.n_jobs,
    )

    summary_path = args.output_root / 'study_summary.json'
    write_study_summary(study, summary_path)

    print()
    try:
        best = study.best_trial
        print(f'Best trial: #{best.number}  value={best.value:.4f}')
        print('Best params:')
        for k, v in sorted(best.params.items()):
            print(f'  {k} = {v}')
    except ValueError:
        print('No successful trials.')
    print(f'Summary: {summary_path}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
