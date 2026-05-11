"""Score an RTAB-Map trial run by start-to-end pose drift.

The accuracy metric is the straight-line distance between the first and last
poses in the optimized trajectory. It only makes sense for bags where the
user deliberately drove a loop and returned to (approximately) the same spot;
otherwise the "ground truth" of zero drift is meaningless.

Pipeline per bag:
  1. ``rtabmap-export --poses --poses_format 10`` writes a TUM-format file
     (``stamp x y z qx qy qz qw`` per line) next to the database.
  2. Parse the file → ``TrajectoryStats`` (n_poses, duration, path length,
     start/end xyz, drift_m, drift / path_length).
  3. Write ``metrics.json`` in the same directory.

CLI: ``ros2 run rove_rtabmap_tuner score_trial <trial_dir>`` re-scores every
``rtabmap.db`` it finds under the trial directory and writes an aggregate
``trial_scores.json``. Useful for re-scoring without re-running.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
from dataclasses import dataclass, asdict, field
from pathlib import Path
from typing import Optional


@dataclass
class TrajectoryStats:
    n_poses: int
    duration_s: float
    path_length_m: float
    start_xyz: list[float]
    end_xyz: list[float]
    drift_m: float                    # ||end_xyz - start_xyz||
    drift_per_path: Optional[float]   # drift_m / path_length_m, None if path is ~0


@dataclass
class ScoreResult:
    db_path: str
    success: bool
    stats: Optional[TrajectoryStats] = None
    error: Optional[str] = None


def _find_export_output(work_dir: Path, output_name: str) -> Optional[Path]:
    """rtabmap-export's output filename varies a bit by version; try a few
    patterns and fall back to the broadest glob.
    """
    candidates = list(work_dir.glob(f'{output_name}_poses.txt'))
    if candidates:
        return candidates[0]
    candidates = list(work_dir.glob(f'{output_name}.txt'))
    if candidates:
        return candidates[0]
    candidates = sorted(work_dir.glob('*poses*.txt'))
    return candidates[0] if candidates else None


def export_tum_trajectory(db_path: Path, output_dir: Path) -> Path:
    """Run ``rtabmap-export`` to produce a TUM-format trajectory file at
    ``<output_dir>/trajectory.tum``. Raises ``RuntimeError`` if the database
    has no poses or the export fails.
    """
    output_dir.mkdir(parents=True, exist_ok=True)
    output_name = 'trajectory'
    cmd = [
        'rtabmap-export',
        '--poses',
        '--poses_format', '10',
        '--output_dir', str(output_dir),
        '--output', output_name,
        str(db_path),
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        # rtabmap-export prints errors to stdout, not stderr.
        msg = (result.stdout + result.stderr).strip().splitlines()
        tail = msg[-1] if msg else '(no output)'
        raise RuntimeError(f'rtabmap-export failed (code {result.returncode}): {tail}')

    out = _find_export_output(output_dir, output_name)
    if out is None:
        raise RuntimeError(
            f'rtabmap-export succeeded but produced no _poses.txt file in {output_dir}'
        )

    canonical = output_dir / 'trajectory.tum'
    if out != canonical:
        canonical.write_text(out.read_text())
    return canonical


def parse_tum(path: Path) -> list[tuple[float, ...]]:
    """Parse TUM lines: ``timestamp tx ty tz qx qy qz qw``. Silently skips
    blank/comment lines and any line with fewer than 8 numeric columns.
    """
    poses: list[tuple[float, ...]] = []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        parts = line.split()
        if len(parts) < 8:
            continue
        try:
            poses.append(tuple(float(p) for p in parts[:8]))
        except ValueError:
            continue
    return poses


def compute_stats(poses: list[tuple[float, ...]]) -> TrajectoryStats:
    if not poses:
        return TrajectoryStats(
            n_poses=0, duration_s=0.0, path_length_m=0.0,
            start_xyz=[0.0, 0.0, 0.0], end_xyz=[0.0, 0.0, 0.0],
            drift_m=0.0, drift_per_path=None,
        )

    first = poses[0]
    last = poses[-1]
    start_xyz = [first[1], first[2], first[3]]
    end_xyz = [last[1], last[2], last[3]]

    dx, dy, dz = end_xyz[0] - start_xyz[0], end_xyz[1] - start_xyz[1], end_xyz[2] - start_xyz[2]
    drift_m = math.sqrt(dx * dx + dy * dy + dz * dz)

    path_length = 0.0
    for i in range(1, len(poses)):
        ddx = poses[i][1] - poses[i - 1][1]
        ddy = poses[i][2] - poses[i - 1][2]
        ddz = poses[i][3] - poses[i - 1][3]
        path_length += math.sqrt(ddx * ddx + ddy * ddy + ddz * ddz)

    return TrajectoryStats(
        n_poses=len(poses),
        duration_s=last[0] - first[0],
        path_length_m=path_length,
        start_xyz=start_xyz,
        end_xyz=end_xyz,
        drift_m=drift_m,
        drift_per_path=(drift_m / path_length) if path_length > 1e-6 else None,
    )


def score_run(db_path: Path, output_dir: Path) -> ScoreResult:
    """Score one bag run: extract trajectory, compute stats, write metrics.json.

    Always returns a ScoreResult — no exceptions to the caller. Failures are
    captured in ``ScoreResult.error``.
    """
    db_path = Path(db_path)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    try:
        if not db_path.exists():
            raise RuntimeError(f'database does not exist: {db_path}')
        traj_path = export_tum_trajectory(db_path, output_dir)
        poses = parse_tum(traj_path)
        stats = compute_stats(poses)
        result = ScoreResult(db_path=str(db_path), success=True, stats=stats)
    except Exception as exc:  # noqa: BLE001 — scoring failure must not crash the trial
        result = ScoreResult(db_path=str(db_path), success=False, error=str(exc))

    (output_dir / 'metrics.json').write_text(
        json.dumps(asdict(result), indent=2, sort_keys=True)
    )
    return result


def score_trial(trial_dir: Path) -> dict:
    """Re-score every ``rtabmap.db`` found under ``trial_dir``. Writes
    ``trial_scores.json`` with per-bag results and an aggregate summary.
    """
    trial_dir = Path(trial_dir)
    per_bag: dict[str, dict] = {}

    for db in sorted(trial_dir.glob('*/rtabmap.db')):
        bag_dir = db.parent
        result = score_run(db, bag_dir)
        per_bag[bag_dir.name] = asdict(result)

    successful_drifts = [
        s['stats']['drift_m']
        for s in per_bag.values()
        if s['success'] and s['stats'] is not None
    ]
    aggregate = {
        'n_bags': len(per_bag),
        'n_successful': len(successful_drifts),
        'mean_drift_m': (sum(successful_drifts) / len(successful_drifts))
            if successful_drifts else None,
        'max_drift_m': max(successful_drifts) if successful_drifts else None,
        'min_drift_m': min(successful_drifts) if successful_drifts else None,
    }

    output = {'per_bag': per_bag, 'aggregate': aggregate}
    (trial_dir / 'trial_scores.json').write_text(json.dumps(output, indent=2, sort_keys=True))
    return output


def _format_bag_line(name: str, score: dict) -> str:
    if not score['success']:
        return f'  {name}: FAIL — {score["error"]}'
    s = score['stats']
    ratio = f'{s["drift_per_path"] * 100:.2f}%' if s['drift_per_path'] is not None else 'n/a'
    return (
        f'  {name}: drift={s["drift_m"]:.3f}m  path={s["path_length_m"]:.2f}m  '
        f'ratio={ratio}  n_poses={s["n_poses"]}'
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('trial_dir', type=Path, help='Path to a trial output directory.')
    args = parser.parse_args()

    output = score_trial(args.trial_dir)
    print(f'Trial: {args.trial_dir}')
    for name in sorted(output['per_bag']):
        print(_format_bag_line(name, output['per_bag'][name]))
    agg = output['aggregate']
    if agg['n_successful']:
        print(f'  Aggregate: mean={agg["mean_drift_m"]:.3f}m  '
              f'min={agg["min_drift_m"]:.3f}m  max={agg["max_drift_m"]:.3f}m  '
              f'({agg["n_successful"]}/{agg["n_bags"]} bags successful)')
    else:
        print(f'  Aggregate: 0/{agg["n_bags"]} bags successful')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
