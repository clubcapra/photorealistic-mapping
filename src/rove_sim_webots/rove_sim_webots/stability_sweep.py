"""Run verify_trajectory across all worlds N times in parallel batches and
aggregate the per-world variance statistics.

Goal: confirm trajectories are reproducible — same world should give
similar actual path length, min clearance, deviation, and collision
counts run-to-run.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

WORLDS = ['outdoor_terrain', 'indoor_warehouse', 'outdoor_rocky',
          'outdoor_urban', 'indoor_structured', 'indoor_office', 'mixed']


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--reps', type=int, default=3)
    p.add_argument('--batch-size', type=int, default=3)
    p.add_argument('--out-dir', type=Path,
                    default=Path.home() / 'overnight_runs' / 'stability_sweep')
    p.add_argument('--worlds', nargs='+', default=WORLDS)
    args = p.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    print(f'[stability] {len(args.worlds)} worlds x {args.reps} reps in '
          f'batches of {args.batch_size}', flush=True)

    t0 = time.monotonic()
    all_runs = []
    for rep in range(args.reps):
        rep_dir = args.out_dir / f'rep_{rep}'
        rep_dir.mkdir(parents=True, exist_ok=True)
        print(f'\n[stability] rep {rep + 1}/{args.reps}', flush=True)
        env = os.environ.copy()
        env['PYTHONUNBUFFERED'] = '1'
        env['BASE_DOMAIN'] = str(130 + rep * 20)
        env['BASE_PORT'] = str(1240 + rep * 20)
        cmd = ['python3', '-u', '-m', 'rove_sim_webots.parallel_verify',
               '--worlds', *args.worlds,
               '--batch-size', str(args.batch_size),
               '--out-dir', str(rep_dir),
               '--base-domain', str(130 + rep * 20),
               '--base-port', str(1240 + rep * 20)]
        rep_log = rep_dir / 'rep.log'
        with rep_log.open('w') as f:
            rc = subprocess.call(cmd, env=env, stdout=f, stderr=subprocess.STDOUT)
        summary_path = rep_dir / 'parallel_summary.json'
        if summary_path.exists():
            d = json.loads(summary_path.read_text())
            for r in d['results']:
                r['rep'] = rep
                all_runs.append(r)
        print(f'  rep {rep} done rc={rc}', flush=True)

    elapsed = time.monotonic() - t0

    # Aggregate by world
    by_world: dict[str, list] = {}
    for r in all_runs:
        by_world.setdefault(r['world'], []).append(r)

    print(f'\n=== stability summary (total {elapsed:.0f}s) ===', flush=True)
    print(f"{'world':18s}  {'reps':>4s}  {'pass':>4s}  "
          f"{'actual_m mean+/-sd':>20s}  "
          f"{'min_clear mean+/-sd':>22s}  {'coll_max':>9s}", flush=True)

    summary = []
    for w, runs in by_world.items():
        actuals = [r.get('actual_length_m', 0) for r in runs]
        clears = [r.get('min_actual_clearance_m', 0) for r in runs]
        colls = [r.get('collisions', 0) for r in runs]
        passes = sum(1 for r in runs if r.get('status') == 'pass')
        am = sum(actuals) / len(actuals)
        asd = (sum((x - am) ** 2 for x in actuals) / len(actuals)) ** 0.5
        cm = sum(clears) / len(clears)
        csd = (sum((x - cm) ** 2 for x in clears) / len(clears)) ** 0.5
        summary.append({
            'world': w, 'reps': len(runs), 'passes': passes,
            'actual_mean_m': am, 'actual_sd_m': asd,
            'clearance_mean_m': cm, 'clearance_sd_m': csd,
            'collisions_max': max(colls) if colls else 0,
        })
        print(f"{w:18s}  {len(runs):>4d}  {passes:>4d}  "
              f"{am:>9.1f}+/-{asd:>5.1f}m  "
              f"{cm:>9.2f}+/-{csd:>6.2f}m  {max(colls):>9d}",
              flush=True)

    out_path = args.out_dir / 'stability_summary.json'
    out_path.write_text(json.dumps({
        'elapsed_s': elapsed,
        'reps': args.reps,
        'batch_size': args.batch_size,
        'summary': summary,
        'all_runs': all_runs,
    }, indent=2))
    print(f'\nwrote {out_path}', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
