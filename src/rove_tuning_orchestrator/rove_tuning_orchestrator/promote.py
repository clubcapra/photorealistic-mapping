"""Promote the top-K phase-1 (sim) candidates into a phase-2 (real bag) eval.

Read each stage's Optuna journal, fish out the top-K trials by score, merge
their full param sets (carryover + stage params), and run RealEvaluator on
each. Writes a summary.json that ranks the candidates by their real-bag
score.

CLI:
    promote \
        --root /shared/studies \
        --project capra_v1 \
        --k 5 \
        --bags ~/bags/moving_long_bag1 ~/bags/turning_bag1 \
        --reference ~/bags/reference_clean.db
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
from dataclasses import asdict
from pathlib import Path
from typing import Any, Dict, List

from rove_tuning_orchestrator.evaluators.real import (
    RealEvaluator, RealEvaluatorConfig,
)
from rove_tuning_orchestrator.storage import journal_storage
import optuna


log = logging.getLogger('promote')


def top_k_candidates(
    phase1_dir: Path, k: int,
) -> List[Dict[str, Any]]:
    """Return the top-K trials across ALL phase-1 stages by score."""
    candidates: List[Dict[str, Any]] = []
    # Find every journal file in phase1.
    for journal in sorted(phase1_dir.glob('*.journal')):
        study_name = journal.stem
        storage = journal_storage(phase1_dir, study_name)
        try:
            study = optuna.load_study(study_name=study_name, storage=storage)
        except (KeyError, ValueError):
            log.warning(f'  skipped {journal} — could not load study')
            continue
        for t in study.trials:
            if t.state != optuna.trial.TrialState.COMPLETE:
                continue
            if t.value is None:
                continue
            full = dict(t.user_attrs.get('carryover') or {})
            full.update(t.params)
            candidates.append({
                'stage': study_name,
                'trial_number': t.number,
                'score': float(t.value),
                'full_params': full,
                'user_attrs': dict(t.user_attrs),
            })
    candidates.sort(key=lambda c: c['score'])
    return candidates[:k]


def run_phase2(
    project_root: Path,
    project_name: str,
    k: int,
    bags: List[Path],
    reference_db: Path,
    real_cfg_overrides: Dict[str, Any],
) -> Dict[str, Any]:
    project = project_root / project_name
    phase1_dir = project / 'phase1_sim'
    phase2_dir = project / 'phase2_real'
    phase2_dir.mkdir(parents=True, exist_ok=True)
    eval_dir = phase2_dir / 'eval'
    eval_dir.mkdir(parents=True, exist_ok=True)

    candidates = top_k_candidates(phase1_dir, k)
    if not candidates:
        raise SystemExit('No completed phase-1 trials found — run phase 1 first.')
    (phase2_dir / 'candidates.json').write_text(
        json.dumps(candidates, indent=2, default=str)
    )

    real_cfg = RealEvaluatorConfig(
        bags=[Path(b).expanduser().resolve() for b in bags],
        reference_db=Path(reference_db).expanduser().resolve(),
        **real_cfg_overrides,
    )
    evaluator = RealEvaluator(real_cfg)

    results: List[Dict[str, Any]] = []
    for idx, cand in enumerate(candidates):
        out_dir = eval_dir / f'candidate_{idx:02d}'
        log.info(
            f'=== candidate {idx} (phase1 score={cand["score"]:.4f}, '
            f'from {cand["stage"]}/trial_{cand["trial_number"]:04d}) ==='
        )
        result = evaluator.evaluate(
            params=cand['full_params'],
            trial_id=f'phase2/cand_{idx:02d}',
            out_dir=out_dir,
        )
        entry = {
            'candidate_idx': idx,
            'phase1_origin': f'{cand["stage"]}/trial_{cand["trial_number"]:04d}',
            'phase1_score': cand['score'],
            'phase2_score': result.score,
            'phase2_failed': result.failed,
            'phase2_failure_reason': result.failure_reason,
            'metrics': result.metrics,
            'params': cand['full_params'],
        }
        results.append(entry)
        log.info(f'  phase2 score = {result.score:.4f}')

    results.sort(key=lambda r: r['phase2_score'])
    summary = {
        'project': project_name,
        'k': k,
        'bags': [str(b) for b in real_cfg.bags],
        'reference_db': str(real_cfg.reference_db),
        'ranked_results': results,
        'recommended': results[0] if results else None,
    }
    (phase2_dir / 'summary.json').write_text(
        json.dumps(summary, indent=2, default=str)
    )
    return summary


def main() -> int:
    p = argparse.ArgumentParser(prog='promote')
    p.add_argument('--root', required=True, help='Studies root.')
    p.add_argument('--project', required=True)
    p.add_argument('--k', type=int, default=5)
    p.add_argument('--bags', nargs='+', required=True)
    p.add_argument('--reference', required=True,
                   help='Path to the cleaned-up reference rtabmap.db')
    p.add_argument('--tau', type=float, default=0.5,
                   help='Node-correspondence distance threshold (m).')
    p.add_argument('--corr-weight', type=float, default=1.0)
    p.add_argument('--loss-weight', type=float, default=2.0)
    p.add_argument('--aggregator', default='median',
                   choices=('mean', 'median', 'max'))
    p.add_argument('--max-bag-duration-s', type=float, default=180.0)
    p.add_argument('--timeout-s', type=float, default=600.0)
    p.add_argument('--domain-id', type=int, default=123)
    p.add_argument('--min-correspondence-ratio', type=float, default=0.05)
    p.add_argument('--log-level', default='INFO')
    args = p.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format='%(asctime)s %(levelname)s %(name)s: %(message)s',
    )

    summary = run_phase2(
        project_root=Path(args.root).expanduser().resolve(),
        project_name=args.project,
        k=args.k,
        bags=args.bags,
        reference_db=args.reference,
        real_cfg_overrides=dict(
            tau_m=args.tau,
            corr_weight=args.corr_weight,
            loss_weight=args.loss_weight,
            aggregator=args.aggregator,
            max_bag_duration_s=args.max_bag_duration_s,
            timeout_s=args.timeout_s,
            domain_id=args.domain_id,
            min_correspondence_ratio=args.min_correspondence_ratio,
        ),
    )
    print(json.dumps(summary, indent=2, default=str))
    return 0


if __name__ == '__main__':
    sys.exit(main())
