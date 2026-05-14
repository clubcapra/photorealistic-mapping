"""Per-bag diagnostics for an Optuna study output directory.

What it surfaces (using only existing trial data, no new runs):

  1. Which bag(s) most often dominate the worst-drift score? (the "blocker
     bags" — fixing them or removing them gives the biggest jump.)
  2. Each bag's achievable drift range — does ANY param set get this bag
     to low drift, or is it structurally hard?
  3. Per-bag winning trials — which trial achieves the best score on each
     bag, and how many *distinct* winning trials there are. Few distinct
     winners = a single param set works across bags. Many distinct winners =
     bags need different params (the "no universal champion" diagnosis).
  4. Re-aggregation: what the top trials would have been under median, q75,
     q90, and max aggregators (so you can compare aggregator choices on the
     same data).

Run with the study output directory as the only argument:

    ros2 run rove_rtabmap_tuner analyze_per_bag /home/iliana/prog/study_full

Read-only — never writes to the DB or trial dirs.
"""

from __future__ import annotations

import argparse
import json
import statistics
from collections import Counter, defaultdict
from pathlib import Path


def _quantile(values: list[float], q: float) -> float:
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


def _gather_trials(study_dir: Path) -> list[dict]:
    """Walk trial_NNNN/<bag>/metrics.json and return [{num, bag_drifts}, ...]."""
    out: list[dict] = []
    for child in sorted(study_dir.iterdir()):
        if not child.is_dir() or not child.name.startswith('trial_'):
            continue
        try:
            num = int(child.name.split('_')[1])
        except (IndexError, ValueError):
            continue
        bag_drifts: dict[str, float] = {}
        for bd in sorted(p for p in child.iterdir() if p.is_dir()):
            mj = bd / 'metrics.json'
            if not mj.exists():
                continue
            try:
                m = json.loads(mj.read_text())
            except json.JSONDecodeError:
                continue
            if not m.get('success'):
                continue
            stats = m.get('stats') or {}
            d = stats.get('drift_per_path')
            if d is not None:
                bag_drifts[bd.name] = float(d)
        if bag_drifts:
            out.append({'num': num, 'bag_drifts': bag_drifts})
    return out


def report(study_dir: Path, min_bags_for_rank: int = 5) -> None:
    trials = _gather_trials(study_dir)
    if not trials:
        print(f'no trial data found under {study_dir}')
        return
    print(f'analyzed {len(trials)} trials with per-bag data\n')

    # Q1: most often the worst-drift bag?
    worst_counter: Counter = Counter()
    for t in trials:
        worst = max(t['bag_drifts'].items(), key=lambda kv: kv[1])[0]
        worst_counter[worst] += 1
    print('=== which bag is most often the worst-drift bag? ===')
    print(f'  {"bag":<32}{"times worst":<14}{"% of trials"}')
    for bag, n in worst_counter.most_common():
        print(f'  {bag:<32}{n:<14}{n * 100 // len(trials)}%')

    # Q2: per-bag drift achievability
    per_bag: dict[str, list[float]] = defaultdict(list)
    for t in trials:
        for bag, d in t['bag_drifts'].items():
            per_bag[bag].append(d)

    print('\n=== per-bag drift achievability (the floor tells you whether the bag is solvable) ===')
    print(f'  {"bag":<32}{"n":<6}{"min":<10}{"median":<10}{"q75":<10}{"max":<10}')
    for bag in sorted(per_bag):
        ds = sorted(per_bag[bag])
        if len(ds) < 4:
            continue
        print(f'  {bag:<32}{len(ds):<6}{ds[0]:<10.4f}{statistics.median(ds):<10.4f}'
              f'{_quantile(ds, 0.75):<10.4f}{ds[-1]:<10.4f}')

    # Q3: per-bag winning trials → no-magic-param-set check
    winners: dict[str, tuple[int, float]] = {}
    for bag in per_bag:
        candidates = [(t['num'], t['bag_drifts'].get(bag, 1.0)) for t in trials if bag in t['bag_drifts']]
        winners[bag] = min(candidates, key=lambda x: x[1])
    distinct_winners = sorted({w[0] for w in winners.values()})
    print('\n=== per-bag winning trials (different bags want different params?) ===')
    for bag in sorted(winners):
        t_num, val = winners[bag]
        print(f'  {bag:<32}: best by trial #{t_num} with drift={val:.4f}')
    print(f'  → {len(distinct_winners)} distinct winning trials for {len(winners)} bags'
          f' (low ratio = bags want similar params; high ratio = no universal champion)')
    print(f'  → winning trial numbers: {distinct_winners}')

    # Q4: top trials by each aggregator (only trials with >= min_bags_for_rank bags)
    eligible = [
        {
            'num': t['num'],
            'n_bags': len(t['bag_drifts']),
            'drifts': list(t['bag_drifts'].values()),
        }
        for t in trials if len(t['bag_drifts']) >= min_bags_for_rank
    ]
    if eligible:
        for agg_name, agg_fn in [
            ('median', statistics.median),
            ('q75', lambda v: _quantile(v, 0.75)),
            ('q90', lambda v: _quantile(v, 0.90)),
            ('max', max),
        ]:
            print(f'\n=== top 5 by {agg_name} (across trials with >={min_bags_for_rank} bags) ===')
            ranked = sorted(eligible, key=lambda e: agg_fn(e['drifts']))[:5]
            for r in ranked:
                ds = r['drifts']
                print(f"  #{r['num']:>4} (n={r['n_bags']:>2}): "
                      f"median={statistics.median(ds):.4f}  "
                      f"q75={_quantile(ds, 0.75):.4f}  "
                      f"q90={_quantile(ds, 0.90):.4f}  "
                      f"max={max(ds):.4f}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('study_dir', type=Path,
                        help='Optimizer --output-root directory.')
    parser.add_argument('--min-bags', type=int, default=5,
                        help='Minimum scoreable bags for a trial to be ranked (default %(default)s).')
    args = parser.parse_args()
    report(args.study_dir, min_bags_for_rank=args.min_bags)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
