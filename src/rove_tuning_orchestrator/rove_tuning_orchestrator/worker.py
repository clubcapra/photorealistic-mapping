"""Distributed worker — pulls trials from a shared journal, evaluates, pushes.

Optuna's JournalFileStorage allows multiple processes (on the same machine
or across machines via a shared filesystem) to concurrently optimize the
same study. This worker is a thin loop:

    while not stopped:
        trial = study.ask()    # next params from the sampler
        score = evaluator.evaluate(trial.params)
        study.tell(trial, score)

The shared filesystem can be:
- NFS / SSHFS (typical lab setup)
- syncthing-mirrored folder (most no-port-friendly option)
- rclone-mounted S3/B2
- a git repo whose journal you periodically commit/pull (clunky but works)

Run one of these per machine, pointing all of them at the same --root, and
they cooperatively optimize.
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path
from typing import Dict, Optional

import optuna

from rove_tuning_orchestrator.evaluators.sim import SimEvaluator, SimEvaluatorConfig
from rove_tuning_orchestrator import search_spaces
from rove_tuning_orchestrator.storage import create_or_load_study, make_sampler


log = logging.getLogger('worker')


STAGE_FILES = {
    'stage_a_icp_core': 'stage_a_icp_core.yaml',
    'stage_b_icp_corresp': 'stage_b_icp_corresp.yaml',
    'stage_c_odom_filter': 'stage_c_odom_filter.yaml',
    'stage_d_loop_closure': 'stage_d_loop_closure.yaml',
}


def main() -> int:
    p = argparse.ArgumentParser(prog='worker')
    p.add_argument('--root', required=True)
    p.add_argument('--project', required=True)
    p.add_argument('--stage', required=True, choices=list(STAGE_FILES.keys()),
                   help='Which stage this worker contributes to.')
    p.add_argument('--max-trials', type=int, default=10,
                   help='Stop after this many trials (this worker).')
    p.add_argument('--sampler', default='cma_es')
    p.add_argument('--seed', type=int, default=0,
                   help='Per-worker seed; different workers MUST have different seeds.')
    p.add_argument('--sim-world', default='outdoor_terrain.wbt')
    p.add_argument('--sim-trajectory', default='outdoor_loop1')
    p.add_argument('--sim-domain-id', type=int, default=122,
                   help='Different workers on the same MACHINE must have different '
                        'domain IDs (range 120-140); on different machines either works.')
    p.add_argument('--sim-timeout', type=float, default=300.0)
    p.add_argument('--log-level', default='INFO')
    args = p.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format='%(asctime)s %(levelname)s %(name)s: %(message)s',
    )

    phase1_dir = (
        Path(args.root).expanduser().resolve() / args.project / 'phase1_sim'
    )
    artifacts_dir = phase1_dir / 'artifacts' / args.stage
    artifacts_dir.mkdir(parents=True, exist_ok=True)

    sampler = make_sampler(args.sampler, seed=args.seed)
    study = create_or_load_study(
        study_root=phase1_dir,
        study_name=args.stage,
        sampler=sampler,
        direction='minimize',
    )

    # Load the carryover-best for stages > A from best_per_stage.json.
    carryover = _load_carryover(phase1_dir, args.stage)
    space = search_spaces.load_from_package_share(STAGE_FILES[args.stage])

    sim = SimEvaluator(SimEvaluatorConfig(
        world=args.sim_world,
        trajectory=args.sim_trajectory,
        headless=True,
        domain_id=args.sim_domain_id,
        timeout_s=args.sim_timeout,
    ))

    log.info(f'worker starting: project={args.project} stage={args.stage} '
             f'domain={args.sim_domain_id} max_trials={args.max_trials}')

    for i in range(args.max_trials):
        trial = study.ask()
        # Suggest each param using the spec.
        stage_params = {}
        for spec in space.params:
            stage_params[spec.name] = spec.suggest(trial)
        full = search_spaces.merge_params(carryover, stage_params)
        trial.set_user_attr('carryover', carryover)
        trial.set_user_attr('stage', args.stage)
        trial.set_user_attr('worker_seed', args.seed)

        out = artifacts_dir / f'trial_{trial.number:04d}_worker{args.seed}'
        log.info(f'  [{i+1}/{args.max_trials}] trial #{trial.number} '
                 f'params={stage_params}')
        result = sim.evaluate(
            params=full,
            trial_id=f'{args.stage}/{trial.number:04d}',
            out_dir=out,
        )
        for k, v in result.metrics.items():
            trial.set_user_attr(f'metric.{k}', _jsonable(v))
        for k, v in result.artifacts.items():
            trial.set_user_attr(f'artifact.{k}', str(v))
        if result.failed:
            trial.set_user_attr('failed', True)
            trial.set_user_attr('failure_reason', result.failure_reason)
        study.tell(trial, result.score)
        log.info(f'    score={result.score:.4f}  failed={result.failed}')

    log.info('worker done.')
    return 0


def _load_carryover(phase1_dir: Path, stage: str) -> Dict:
    """Pull best params from prior stages (A→B→C→D ordering)."""
    best_file = phase1_dir / 'best_per_stage.json'
    if not best_file.exists():
        return {}
    data = __import__('json').loads(best_file.read_text())
    order = list(STAGE_FILES.keys())
    if stage not in order:
        return {}
    keep = order[:order.index(stage)]
    out = {}
    for s in keep:
        if s in data:
            out.update(data[s])
    return out


def _jsonable(v):
    if isinstance(v, (int, float, str, bool)) or v is None:
        return v
    if isinstance(v, (list, tuple)):
        return [_jsonable(x) for x in v]
    if isinstance(v, dict):
        return {str(k): _jsonable(val) for k, val in v.items()}
    return str(v)


if __name__ == '__main__':
    sys.exit(main())
