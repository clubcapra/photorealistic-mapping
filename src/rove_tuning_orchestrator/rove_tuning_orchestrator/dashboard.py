"""Launch optuna-dashboard against this project's studies.

Optuna-dashboard reads JournalFileStorage directly — point it at the same
journal file the orchestrator/worker is writing and you see live progress
across all phases.

Typical usage:

    # On the machine you want to monitor from:
    dashboard --root /shared/studies --project capra_v1 --port 8080

    # Then open http://localhost:8080 in a browser.

Since this exposes a port (8080 by default), it is intentionally a *local*
view. For multiple watchers without ports, each watcher runs their own
dashboard process against the (synced) journal file.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path


def main() -> int:
    p = argparse.ArgumentParser(prog='dashboard')
    p.add_argument('--root', required=True)
    p.add_argument('--project', required=True)
    p.add_argument('--phase', choices=('1', '2', 'all'), default='all',
                   help='1: only phase-1 studies, 2: only phase-2, all: both.')
    p.add_argument('--port', type=int, default=8080)
    p.add_argument('--host', default='127.0.0.1')
    p.add_argument('--install-if-missing', action='store_true',
                   help='pip install optuna-dashboard if it is not on PATH.')
    args = p.parse_args()

    project = Path(args.root).expanduser().resolve() / args.project
    if not project.exists():
        raise SystemExit(f'Project not found: {project}')

    if shutil.which('optuna-dashboard') is None:
        if args.install_if_missing:
            subprocess.check_call([sys.executable, '-m', 'pip', 'install',
                                   '--user', 'optuna-dashboard'])
        else:
            raise SystemExit(
                'optuna-dashboard not found. Install via:\n'
                '  pip install --user optuna-dashboard\n'
                'or re-run with --install-if-missing.'
            )

    # JournalFileStorage URL form Optuna's dashboard understands:
    #   journal:///abs/path/to/file.journal
    # Multiple journal files require multiple processes — dashboard supports
    # one storage at a time. We default to phase-1 unless --phase=2; for both,
    # run two instances on different ports.
    if args.phase == '2':
        target_dir = project / 'phase2_real'
    else:
        target_dir = project / 'phase1_sim'
    journals = sorted(target_dir.glob('*.journal'))
    if not journals:
        raise SystemExit(f'No journal files found under {target_dir}')

    # Dashboard takes ONE journal at a time. To view multiple, we spin one
    # subprocess per journal at increasing ports.
    if len(journals) == 1 or args.phase != 'all':
        journal = journals[0]
        cmd = ['optuna-dashboard',
               f'--host={args.host}',
               f'--port={args.port}',
               f'sqlite+journal:///{journal}']
        # Optuna 3.x: storage URL prefix is just "journal://"
        cmd[-1] = f'journal://{journal}'
        print(f'launching: {" ".join(cmd)}')
        print(f'open http://{args.host}:{args.port} in your browser')
        return subprocess.call(cmd)

    # Multiple journals (e.g. one per stage): launch a tab per journal.
    procs = []
    port = args.port
    for j in journals:
        cmd = ['optuna-dashboard', f'--host={args.host}', f'--port={port}',
               f'journal://{j}']
        print(f'  {j.name} -> http://{args.host}:{port}/')
        procs.append(subprocess.Popen(cmd))
        port += 1
    print(f'launched {len(procs)} dashboards. Ctrl-C to stop all.')
    try:
        for p in procs:
            p.wait()
    except KeyboardInterrupt:
        for p in procs:
            p.terminate()
    return 0


if __name__ == '__main__':
    sys.exit(main())
