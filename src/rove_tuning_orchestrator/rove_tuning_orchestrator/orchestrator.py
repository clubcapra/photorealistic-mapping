"""Two-phase RTAB-Map tuning orchestrator.

Phase 1: stages A→B→C→D→E on the simulator (cheap, ground-truth ATE).
Phase 2: top-K candidates from phase 1 are re-evaluated against real bags.

Each stage is its own Optuna study with CMA-ES inside it. Stages share params
through the orchestrator: each stage starts with the best params from the
previous stage and only varies its own subset.

Layout on disk:

    <root>/<project>/
      configs/                       # frozen copies of the YAML used
      phase1_sim/
        stage_a.journal              # one Optuna study per stage
        stage_b.journal
        ...
        stage_e.journal
        best_per_stage.json          # passed forward between stages
        artifacts/<stage>/<trial>/...
      phase2_real/
        candidates.json              # top-K from phase1 (post-promote)
        eval/<candidate_idx>/<bag>/<rtabmap.db, reference_compare.json>
        summary.json
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import sys
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

import optuna

from rove_tuning_orchestrator.evaluators.base import Evaluator
from rove_tuning_orchestrator.evaluators.sim import SimEvaluator, SimEvaluatorConfig
from rove_tuning_orchestrator import search_spaces
from rove_tuning_orchestrator.storage import create_or_load_study, make_sampler


log = logging.getLogger('orchestrator')


STAGE_FILES = [
    ('stage_a_icp_core', 'stage_a_icp_core.yaml'),
    ('stage_b_icp_corresp', 'stage_b_icp_corresp.yaml'),
    ('stage_c_odom_filter', 'stage_c_odom_filter.yaml'),
    ('stage_d_loop_closure', 'stage_d_loop_closure.yaml'),
]


@dataclass
class Phase1Config:
    project_root: Path
    project_name: str
    n_trials_per_stage: int = 20
    sampler: str = 'cma_es'
    seed: int = 17
    stage_yaml_dir: Optional[Path] = None  # default: package share
    sim_cfg: SimEvaluatorConfig = field(default_factory=SimEvaluatorConfig)
    refine_half_width_frac: float = 0.20
    enable_stage_e: bool = True


def run_phase1(cfg: Phase1Config) -> Dict[str, Any]:
    """Run staged sim tuning (A→B→C→D, then optionally joint refine E).

    Returns the best params + stage-by-stage breakdown.
    """
    project_root = cfg.project_root / cfg.project_name
    phase_dir = project_root / 'phase1_sim'
    phase_dir.mkdir(parents=True, exist_ok=True)
    artifacts_dir = phase_dir / 'artifacts'
    artifacts_dir.mkdir(parents=True, exist_ok=True)

    sim = SimEvaluator(cfg.sim_cfg)
    best_per_stage: Dict[str, Dict[str, Any]] = {}
    cumulative_best: Dict[str, Any] = {}

    for stage_name, yaml_filename in STAGE_FILES:
        log.info(f'=== Phase 1 / {stage_name} ===')
        space = _load_space(cfg.stage_yaml_dir, yaml_filename)
        best, study = _run_stage(
            stage_name=stage_name,
            space=space,
            evaluator=sim,
            phase_dir=phase_dir,
            artifacts_dir=artifacts_dir / stage_name,
            cumulative_best=cumulative_best,
            n_trials=cfg.n_trials_per_stage,
            sampler_kind=cfg.sampler,
            seed=cfg.seed,
        )
        best_per_stage[stage_name] = best
        cumulative_best = search_spaces.merge_params(cumulative_best, best)
        _persist(phase_dir / 'best_per_stage.json', best_per_stage)
        log.info(f'  stage best: {best}')

    if cfg.enable_stage_e:
        log.info('=== Phase 1 / stage_e_joint_refine ===')
        refine_space = search_spaces.build_refine_space(
            best_per_stage, half_width_frac=cfg.refine_half_width_frac,
        )
        if refine_space.params:
            best, _ = _run_stage(
                stage_name='stage_e_joint_refine',
                space=refine_space,
                evaluator=sim,
                phase_dir=phase_dir,
                artifacts_dir=artifacts_dir / 'stage_e_joint_refine',
                cumulative_best={},  # E is standalone — its space IS the joint refine
                n_trials=cfg.n_trials_per_stage,
                sampler_kind=cfg.sampler,
                seed=cfg.seed + 1,
            )
            best_per_stage['stage_e_joint_refine'] = best
            cumulative_best = search_spaces.merge_params(cumulative_best, best)
            _persist(phase_dir / 'best_per_stage.json', best_per_stage)

    summary = {
        'project': cfg.project_name,
        'best_per_stage': best_per_stage,
        'final_best_params': cumulative_best,
        'sampler': cfg.sampler,
        'n_trials_per_stage': cfg.n_trials_per_stage,
    }
    _persist(phase_dir / 'summary.json', summary)
    log.info('=== Phase 1 complete ===')
    log.info(f'final best: {cumulative_best}')
    return summary


def _run_stage(
    stage_name: str,
    space: search_spaces.SearchSpace,
    evaluator: Evaluator,
    phase_dir: Path,
    artifacts_dir: Path,
    cumulative_best: Dict[str, Any],
    n_trials: int,
    sampler_kind: str,
    seed: int,
):
    """Run one stage's Optuna optimization."""
    sampler = make_sampler(sampler_kind, seed=seed)
    study = create_or_load_study(
        study_root=phase_dir,
        study_name=stage_name,
        sampler=sampler,
        direction='minimize',
    )
    artifacts_dir.mkdir(parents=True, exist_ok=True)

    def objective(trial: optuna.Trial) -> float:
        stage_params = space.suggest_all(trial)
        # Merge with carry-over best from earlier stages — those params are
        # fixed during THIS stage.
        full_params = search_spaces.merge_params(cumulative_best, stage_params)
        trial.set_user_attr('stage', stage_name)
        trial.set_user_attr('carryover', dict(cumulative_best))

        trial_out = artifacts_dir / f'trial_{trial.number:04d}'
        result = evaluator.evaluate(
            params=full_params,
            trial_id=f'{stage_name}/{trial.number:04d}',
            out_dir=trial_out,
        )
        for k, v in result.metrics.items():
            trial.set_user_attr(f'metric.{k}', _jsonable(v))
        for k, v in result.artifacts.items():
            trial.set_user_attr(f'artifact.{k}', str(v))
        if result.warnings:
            trial.set_user_attr('warnings', result.warnings)
        if result.failed:
            trial.set_user_attr('failed', True)
            trial.set_user_attr('failure_reason', result.failure_reason)
            log.warning(
                f'  trial {trial.number} FAILED: {result.failure_reason}'
            )
        else:
            log.info(
                f'  trial {trial.number}  score={result.score:.4f}  '
                f'metrics={result.metrics}'
            )
        return result.score

    study.optimize(objective, n_trials=n_trials, show_progress_bar=False)
    return study.best_params, study


def _load_space(stage_yaml_dir: Optional[Path], filename: str):
    if stage_yaml_dir is not None:
        return search_spaces.load(stage_yaml_dir / filename)
    return search_spaces.load_from_package_share(filename)


def _persist(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, default=_jsonable))


def _jsonable(v):
    if isinstance(v, (int, float, str, bool)) or v is None:
        return v
    if isinstance(v, (list, tuple)):
        return [_jsonable(x) for x in v]
    if isinstance(v, dict):
        return {str(k): _jsonable(val) for k, val in v.items()}
    return str(v)


# ---------- CLI ----------

def main() -> int:
    p = argparse.ArgumentParser(prog='tune')
    p.add_argument('--root', required=True,
                   help='Studies root (a shared directory if using distributed mode)')
    p.add_argument('--project', required=True,
                   help='Project name (subdir under --root). One project = one tuning campaign.')
    p.add_argument('--phase', choices=('1', '2', 'both'), default='1',
                   help='1: sim staged tune. 2: re-eval top-K on real bags (see promote tool). both: chain.')
    p.add_argument('--n-trials-per-stage', type=int, default=20)
    p.add_argument('--sampler', default='cma_es', choices=('cma_es', 'tpe', 'random'))
    p.add_argument('--seed', type=int, default=17)
    p.add_argument('--sim-world', default='outdoor_terrain.wbt')
    p.add_argument('--sim-trajectory', default='outdoor_loop1')
    p.add_argument('--sim-headless', action='store_true', default=True)
    p.add_argument('--sim-domain-id', type=int, default=122)
    p.add_argument('--sim-webots-port', type=int, default=1234,
                   help='Per-orchestrator Webots IPC port. MUST be unique '
                        'across concurrent orchestrators on the same machine.')
    p.add_argument('--sim-timeout', type=float, default=300.0)
    p.add_argument('--enable-stage-e', action='store_true', default=True)
    p.add_argument('--refine-half-width', type=float, default=0.20)
    p.add_argument('--log-level', default='INFO')
    args = p.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format='%(asctime)s %(levelname)s %(name)s: %(message)s',
    )

    if args.phase not in ('1', 'both'):
        print('Phase 2 is driven by the `promote` tool — see README.', file=sys.stderr)
        return 2

    cfg = Phase1Config(
        project_root=Path(args.root).expanduser().resolve(),
        project_name=args.project,
        n_trials_per_stage=args.n_trials_per_stage,
        sampler=args.sampler,
        seed=args.seed,
        sim_cfg=SimEvaluatorConfig(
            world=args.sim_world,
            trajectory=args.sim_trajectory,
            headless=args.sim_headless,
            domain_id=args.sim_domain_id,
            webots_port=args.sim_webots_port,
            timeout_s=args.sim_timeout,
        ),
        refine_half_width_frac=args.refine_half_width,
        enable_stage_e=args.enable_stage_e,
    )
    summary = run_phase1(cfg)

    if args.phase == 'both':
        print('NOTE: phase 2 (real-bag) needs paths to bags + reference db.', file=sys.stderr)
        print('      Use the `promote` tool: rove_tuning_orchestrator/promote.py', file=sys.stderr)

    print(json.dumps(summary, indent=2, default=_jsonable))
    return 0


if __name__ == '__main__':
    sys.exit(main())
