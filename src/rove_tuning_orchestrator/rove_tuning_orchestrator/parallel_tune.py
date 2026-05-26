"""N-parallel tuning sweep against a recorded sim bag.

Skips the full orchestrator (which is single-threaded) and runs N
SimBagEvaluator trials concurrently per round. Each trial uses a unique
ROS_DOMAIN_ID so multiple rtabmap instances don't interfere.

Usage:
    python3 -m rove_tuning_orchestrator.parallel_tune \\
        --bag ~/overnight_runs/bag_replay_tuning/sim_bags/rocky/bag \\
        --search-space stage_a_icp_core \\
        --n-trials 40 --n-parallel 4 \\
        --out-dir ~/overnight_runs/bag_replay_tuning/sweep_rocky_a
"""
from __future__ import annotations

import argparse
import json
import multiprocessing as mp
import os
import sys
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any, Dict, List, Optional

from rove_tuning_orchestrator import search_spaces
from rove_tuning_orchestrator.evaluators.sim_bag import (
    SimBagEvaluator, SimBagEvaluatorConfig,
)


def _sample_params(space, rng) -> Dict[str, Any]:
    """Random sample from the search space (lightweight, no Optuna dep)."""
    out: Dict[str, Any] = {}
    for p in space.params:
        if p.kind == 'float':
            if p.log:
                import math
                lo, hi = math.log(p.low), math.log(p.high)
                out[p.name] = math.exp(rng.uniform(lo, hi))
            else:
                out[p.name] = rng.uniform(p.low, p.high)
        elif p.kind == 'int':
            out[p.name] = rng.randint(int(p.low), int(p.high))
        elif p.kind == 'categorical':
            out[p.name] = rng.choice(p.choices)
        else:
            raise ValueError(f'unknown kind {p.kind}')
    return out


def _evaluate_one(args):
    bag_path, params, domain_id, out_dir = args
    cfg = SimBagEvaluatorConfig(bag=Path(bag_path), domain_id=domain_id)
    ev = SimBagEvaluator(cfg)
    t0 = time.monotonic()
    try:
        result = ev.evaluate(params, f'trial_{out_dir.name}', out_dir)
    except Exception as e:
        return {
            'trial': out_dir.name, 'failed': True,
            'failure_reason': f'evaluator raised: {e!r}',
            'params': params, 'wall_s': time.monotonic() - t0,
        }
    return {
        'trial': out_dir.name,
        'score': result.score,
        'failed': result.failed,
        'failure_reason': result.failure_reason,
        'metrics': result.metrics,
        'params': params,
        'wall_s': time.monotonic() - t0,
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--bag', type=Path, required=True)
    p.add_argument('--search-space', required=True,
                    help='Stage name (without .yaml) under search_spaces/.')
    p.add_argument('--n-trials', type=int, default=40)
    p.add_argument('--n-parallel', type=int, default=4)
    p.add_argument('--seed', type=int, default=17)
    p.add_argument('--out-dir', type=Path, required=True)
    p.add_argument('--base-domain', type=int, default=130)
    p.add_argument('--fixed-params-from', type=Path, default=None,
                    help='JSON file of params held fixed (e.g. carryover from '
                          'a previous stage). Overrides sampled values.')
    args = p.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)

    space = search_spaces.load_from_package_share(f'{args.search_space}.yaml')
    fixed: Dict[str, Any] = {}
    if args.fixed_params_from:
        fixed = json.loads(args.fixed_params_from.read_text())

    # Static reproducibility — single-threaded RNG.
    import random
    rng = random.Random(args.seed)

    # Always evaluate the "defaults" baseline as trial 0 if every param has one.
    baseline = search_spaces.all_defaults(space)
    candidate_params: List[Dict[str, Any]] = []
    if baseline is not None:
        merged = {**baseline, **fixed}
        candidate_params.append(merged)
    while len(candidate_params) < args.n_trials:
        sampled = _sample_params(space, rng)
        merged = {**sampled, **fixed}
        candidate_params.append(merged)
    candidate_params = candidate_params[:args.n_trials]

    print(f'[parallel-tune] {len(candidate_params)} trials in batches of '
          f'{args.n_parallel} against {args.bag}', flush=True)
    t_start = time.monotonic()
    results: List[dict] = []
    summary_path = args.out_dir / 'sweep_summary.json'

    for batch_start in range(0, len(candidate_params), args.n_parallel):
        batch = candidate_params[batch_start:batch_start + args.n_parallel]
        round_idx = batch_start // args.n_parallel
        print(f'\n[round {round_idx + 1}/'
              f'{(len(candidate_params) + args.n_parallel - 1) // args.n_parallel}]',
              flush=True)
        round_args = []
        for i, params in enumerate(batch):
            trial_idx = batch_start + i
            out_dir = args.out_dir / f'trial_{trial_idx:04d}'
            domain_id = args.base_domain + i  # i ∈ [0, n_parallel)
            round_args.append((args.bag, params, domain_id, out_dir))

        with mp.Pool(processes=len(batch)) as pool:
            batch_results = pool.map(_evaluate_one, round_args)
        for r in batch_results:
            results.append(r)
            score = r.get('score', float('inf'))
            failed = r.get('failed', False)
            tag = '✗' if failed else '✓'
            print(f'  {tag} {r["trial"]}: score={score:.4f}  wall={r["wall_s"]:.0f}s',
                  flush=True)
        # Incremental save
        summary_path.write_text(json.dumps({
            'bag': str(args.bag), 'search_space': args.search_space,
            'n_trials': len(results), 'n_parallel': args.n_parallel,
            'elapsed_s': time.monotonic() - t_start,
            'results': results,
        }, indent=2))

    # Final summary + best
    valid = [r for r in results if not r.get('failed') and r.get('score') is not None]
    valid.sort(key=lambda r: r['score'])
    if valid:
        best = valid[0]
        print(f'\nbest score: {best["score"]:.4f}  trial={best["trial"]}', flush=True)
        (args.out_dir / 'best_params.json').write_text(
            json.dumps(best.get('params', {}), indent=2)
        )

    return 0


if __name__ == '__main__':
    sys.exit(main())
