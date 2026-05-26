"""Run multiple verify_trajectory instances in parallel and measure
aggregate throughput. Each instance gets its own ROS_DOMAIN_ID, WEBOTS_PORT,
and xvfb display (auto-picked by xvfb-run -a).

Usage:
    python3 -m rove_sim_webots.parallel_verify \
        --worlds outdoor_terrain indoor_warehouse \
        --out-dir ~/overnight_runs/parallel_test
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path


def _spawn(world: str, out_dir: Path, domain_id: int, port: int,
            sim_mode: str) -> tuple[subprocess.Popen, Path]:
    log_path = out_dir / f'instance_{domain_id}.log'
    env = os.environ.copy()
    env['PYTHONUNBUFFERED'] = '1'
    cmd = ['python3', '-u', '-m', 'rove_sim_webots.verify_trajectory',
           '--worlds', world,
           '--out-dir', str(out_dir / f'instance_{domain_id}'),
           '--domain-id', str(domain_id),
           '--sim-mode', sim_mode]
    env['WEBOTS_PORT'] = str(port)
    # NOTE: skipping the global _cleanup_webots between launches is handled
    # via env; we patch verify_world to skip via env var.
    env['VERIFY_SKIP_CLEANUP'] = '1'
    proc = subprocess.Popen(
        cmd, env=env,
        stdout=open(log_path, 'w'), stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )
    return proc, log_path


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--worlds', nargs='+', required=True,
                    help='Worlds to verify; will be processed in --batch-size chunks.')
    p.add_argument('--batch-size', type=int, default=0,
                    help='Max parallel instances per batch (0 = all in one batch).')
    p.add_argument('--out-dir', type=Path, required=True)
    p.add_argument('--sim-mode', choices=('realtime', 'fast'), default='realtime')
    p.add_argument('--base-domain', type=int, default=130,
                    help='ROS_DOMAIN_IDs start at this value, +1 per instance.')
    p.add_argument('--base-port', type=int, default=1240,
                    help='WEBOTS_PORTs start here, +1 per instance.')
    args = p.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    batch_size = args.batch_size or len(args.worlds)
    print(f'[parallel] {len(args.worlds)} worlds in batches of {batch_size}',
          flush=True)

    all_results = []
    t_total = time.monotonic()
    idx_global = 0
    for batch_start in range(0, len(args.worlds), batch_size):
        batch = args.worlds[batch_start:batch_start + batch_size]
        print(f'[parallel] batch {batch_start // batch_size + 1}: {batch}',
              flush=True)
        procs = []
        t0 = time.monotonic()
        for j, w in enumerate(batch):
            domain = args.base_domain + idx_global
            port = args.base_port + idx_global
            idx_global += 1
            proc, log = _spawn(w, args.out_dir, domain, port, args.sim_mode)
            procs.append((w, domain, port, proc, log))
            print(f'  [{w}] pid={proc.pid} domain={domain} port={port}',
                  flush=True)
            time.sleep(15)
        for w, dom, port, proc, log in procs:
            proc.wait()
            wall = time.monotonic() - t0
            rc = proc.returncode
            summary_path = args.out_dir / f'instance_{dom}' / 'summary.json'
            if summary_path.exists():
                payload = json.loads(summary_path.read_text())
                inner = payload['results'][0] if payload.get('results') else {}
            else:
                inner = {}
            all_results.append({
                'world': w,
                'domain_id': dom,
                'webots_port': port,
                'returncode': rc,
                'wall_clock_s': wall,
                **inner,
            })
            print(f'  [{w}] exited rc={rc} batch_wall={wall:.1f}s', flush=True)
    results = all_results
    aggregate_wall = time.monotonic() - t_total
    print(f'\n[parallel] aggregate wall: {aggregate_wall:.1f}s', flush=True)
    out_summary = args.out_dir / 'parallel_summary.json'
    out_summary.write_text(json.dumps({
        'batch_size': batch_size,
        'n_worlds': len(args.worlds),
        'sim_mode': args.sim_mode,
        'aggregate_wall_s': aggregate_wall,
        'results': results,
    }, indent=2))
    print(f'[parallel] wrote {out_summary}')
    return 0 if all(r['returncode'] == 0 for r in results) else 1


if __name__ == '__main__':
    sys.exit(main())
