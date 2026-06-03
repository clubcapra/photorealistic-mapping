"""Score an RTAB-Map trial run by start-to-end pose drift.

The accuracy metric is the straight-line distance between the first and last
poses in the optimized trajectory. It only makes sense for bags where the
user deliberately drove a loop and returned to (approximately) the same spot;
otherwise the "ground truth" of zero drift is meaningless.

To prevent the optimizer from gaming the metric by producing degenerate
trajectories (1-2 keyframes, ~0 path), ``score_run`` fails any trial whose
trajectory has fewer than ``min_n_poses`` keyframes or shorter than
``min_path_length_m`` of path. Tune these via the ``MIN_*`` module-level
constants.

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
import re
import sqlite3
import subprocess
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Callable, Optional


# Sanity thresholds for trajectory completeness. Below either of these, the
# trajectory is too degenerate to score meaningfully — typically a sign that
# RTAB-Map rejected nearly every scan due to bad params. Treated as failure.
MIN_N_POSES: int = 5
MIN_PATH_LENGTH_M: float = 0.3


@dataclass
class TrajectoryStats:
    n_poses: int
    duration_s: float
    path_length_m: float
    start_xyz: list[float]
    end_xyz: list[float]
    drift_m: float                    # ||end_xyz - start_xyz||
    drift_per_path: Optional[float]   # drift_m / path_length_m, None if path is ~0
    # ICP odometry health: average correspondence ratio over all scan registrations.
    # Higher is better (closer to 1 = scans align cleanly). Parsed from launch.log.
    # None if the log isn't available or has no 'Odom: ratio=' lines.
    mean_icp_ratio: Optional[float] = None
    # Number of accepted loop closures in the SLAM graph (RTAB-Map Link table,
    # types 1-4: global/space/time/user). Higher is better — encodes that the
    # SLAM system recognized revisits and corrected drift accordingly. Zero loop
    # closures across a trajectory that revisits regions is a strong ghosting
    # indicator.
    loop_closure_count: int = 0
    # Map cleanliness: median orthogonal distance from each point to the
    # local-plane fit through its k=20 nearest neighbors in the assembled cloud.
    # A clean wall is ~lidar noise thick (~0.005-0.02 m). A ghosted wall (same
    # surface registered at multiple trajectory poses) is 0.05-0.5+ m thick.
    # Detects ghosting from incorrect loop closures or trajectory bends — but
    # NOT motion under-counting, because if SLAM never advances the keyframe,
    # every scan aligns to the same anchor pose and the local cloud stays thin
    # (just smaller).
    # None if rtabmap-export fails or the cloud is too sparse to score.
    map_thickness_m: Optional[float] = None
    # Bounding-box diagonal of the assembled point cloud (m). Detects the
    # "stuck keyframe" gaming pattern: if SLAM refuses to advance the active
    # keyframe, the cloud is bounded by the lidar range (~10 m max indoors)
    # regardless of how long the robot actually walked. Compare to
    # ``path_length_m``: if path is tiny but extent is large, motion is real
    # but SLAM is under-counting it. Honest trajectory in a 15x15 m area
    # should produce extent ~15-20 m.
    cloud_spatial_extent_m: Optional[float] = None


# Loop-closure link types in rtabmap.db's Link table. Reference:
# rtabmap/corelib/include/rtabmap/core/Link.h enum Type
#   1 = kGlobalClosure, 2 = kLocalSpaceClosure,
#   3 = kLocalTimeClosure, 4 = kUserClosure
_LOOP_CLOSURE_LINK_TYPES: tuple[int, ...] = (1, 2, 3, 4)


def mean_icp_ratio_from_log(launch_log: Path) -> Optional[float]:
    """Parse ``icp_odometry`` 'Odom: ratio=X' lines from a launch.log and
    return the arithmetic mean. None if the log is missing or empty of
    matches.
    """
    if not launch_log.exists():
        return None
    pattern = re.compile(r'Odom: ratio=([0-9.]+)')
    ratios: list[float] = []
    for line in launch_log.read_text(errors='replace').splitlines():
        m = pattern.search(line)
        if m:
            try:
                ratios.append(float(m.group(1)))
            except ValueError:
                continue
    return sum(ratios) / len(ratios) if ratios else None


def _assemble_lidar_cloud(db_path: Path, output_dir: Path):
    """Export the assembled scan-based point cloud and return the loaded
    Open3D point cloud + the downsampled copy used for metric computation.
    Returns (None, None) on any failure.
    """
    import open3d as o3d

    if not db_path.exists():
        return None, None
    cmd = [
        'rtabmap-export', '--cloud', '--scan',
        '--output_dir', str(output_dir),
        '--output', 'map_cloud',
        str(db_path),
    ]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
    except subprocess.TimeoutExpired:
        return None, None
    if result.returncode != 0:
        return None, None

    ply_candidates = list(output_dir.glob('map_cloud*.ply'))
    if not ply_candidates:
        return None, None
    pcd = o3d.io.read_point_cloud(str(ply_candidates[0]))
    if len(pcd.points) < 100:
        return None, None
    pcd_down = pcd.voxel_down_sample(0.005)
    return pcd, pcd_down


def compute_map_thickness_m(db_path: Path, output_dir: Path) -> Optional[float]:
    """Export the assembled point cloud from rtabmap.db and compute the median
    orthogonal distance from each sampled point to the best-fit local plane
    through its k=20 nearest neighbors.

    Lower = cleaner map. ~lidar noise (~0.005-0.02 m) on a clean wall;
    >0.05 m signals ghosting (trajectory error pulling the same surface to
    different locations in the assembled cloud).

    Returns None if rtabmap-export fails, the cloud doesn't get written, or
    the cloud has fewer than 100 points (too sparse to score).

    This metric cannot be gamed by under-reporting motion: a trajectory that
    treats real motion as stationary makes scans from different locations pile
    onto the same coordinates, exploding plane-fit residuals.
    """
    import numpy as np  # local import — scoring is imported in contexts that
    import open3d as o3d  # don't always need open3d

    _pcd_full, pcd_down = _assemble_lidar_cloud(db_path, output_dir)
    if pcd_down is None:
        return None
    points = np.asarray(pcd_down.points)
    if len(points) < 100:
        return None

    tree = o3d.geometry.KDTreeFlann(pcd_down)
    # Sample a subset of points for speed; median of 5000 samples is stable.
    n_samples = min(5000, len(points))
    rng = np.random.default_rng(42)
    sample_idx = rng.choice(len(points), size=n_samples, replace=False)

    thicknesses: list[float] = []
    for idx in sample_idx:
        _k, neighbor_idx, _d = tree.search_knn_vector_3d(points[idx], 20)
        if len(neighbor_idx) < 5:
            continue
        neighbors = points[list(neighbor_idx)]
        centroid = neighbors.mean(axis=0)
        # SVD on centered neighbors. Smallest singular vector is the local plane normal.
        centered = neighbors - centroid
        try:
            _u, _sigma, vh = np.linalg.svd(centered, full_matrices=False)
        except np.linalg.LinAlgError:
            continue
        normal = vh[-1]
        # Orthogonal distance from the query point to the plane through the centroid.
        thicknesses.append(float(abs(np.dot(points[idx] - centroid, normal))))

    if not thicknesses:
        return None
    return float(np.median(thicknesses))


def compute_cloud_spatial_extent_m(db_path: Path, output_dir: Path) -> Optional[float]:
    """Return the bounding-box diagonal of the assembled lidar cloud (m).

    This detects the "stuck keyframe" gaming pattern where SLAM under-counts
    motion. If the trajectory pins everything to one keyframe pose, the
    assembled cloud is bounded by the lidar's effective range (~10 m indoors),
    even if the robot actually walked through a 15+ m space.

    Use alongside ``path_length_m``: a real trajectory in a 15x15 m area
    should produce extent ~15-20 m. If extent is small (≤8 m) on a bag
    where the robot is known to have walked the full space, the SLAM
    trajectory is under-counting motion regardless of what drift_per_path
    reports.

    Returns None on export failure or empty cloud.
    """
    import numpy as np  # noqa: F401 — kept for clarity

    pcd_full, _ = _assemble_lidar_cloud(db_path, output_dir)
    if pcd_full is None:
        return None
    bbox = pcd_full.get_axis_aligned_bounding_box()
    extent = bbox.get_extent()  # [dx, dy, dz]
    return float((extent[0] ** 2 + extent[1] ** 2 + extent[2] ** 2) ** 0.5)


def loop_closure_count_from_db(db_path: Path) -> int:
    """Count loop-closure edges in the rtabmap.db SLAM graph. Returns 0 if
    the DB is missing, lacks a Link table, or any sqlite error occurs.
    """
    if not db_path.exists():
        return 0
    try:
        con = sqlite3.connect(f'file:{db_path}?mode=ro', uri=True)
    except sqlite3.Error:
        return 0
    try:
        placeholders = ','.join('?' * len(_LOOP_CLOSURE_LINK_TYPES))
        row = con.execute(
            f'SELECT COUNT(*) FROM Link WHERE type IN ({placeholders})',
            _LOOP_CLOSURE_LINK_TYPES,
        ).fetchone()
        return int(row[0]) if row else 0
    except sqlite3.Error:
        return 0
    finally:
        con.close()


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
        # Augment with the extra-metric signals. These come from sibling
        # artifacts (launch.log, the DB itself) — they're always populated
        # when available; the optimizer can then ask for whichever one(s)
        # to optimize on.
        stats.mean_icp_ratio = mean_icp_ratio_from_log(output_dir / 'launch.log')
        stats.loop_closure_count = loop_closure_count_from_db(db_path)
        # Map-cleanliness metric. Adds ~10-30 s per bag (rtabmap-export + Open3D
        # plane fits). Run after the cheap metrics so a sparsity failure short-
        # circuits before we pay this cost.
        stats.map_thickness_m = compute_map_thickness_m(db_path, output_dir)
        # Cloud spatial extent — defends against motion under-counting where
        # map_thickness alone can't (everything aligned to one pose looks
        # locally clean but covers only the lidar's range).
        stats.cloud_spatial_extent_m = compute_cloud_spatial_extent_m(db_path, output_dir)
        # Reject degenerate trajectories — they don't measure SLAM accuracy,
        # they measure how aggressively RTAB-Map rejected scans.
        if stats.n_poses < MIN_N_POSES:
            raise RuntimeError(
                f'trajectory too sparse: {stats.n_poses} poses '
                f'(min {MIN_N_POSES}); likely metric gaming'
            )
        if stats.path_length_m < MIN_PATH_LENGTH_M:
            raise RuntimeError(
                f'trajectory too short: {stats.path_length_m:.3f}m path '
                f'(min {MIN_PATH_LENGTH_M}m); likely metric gaming'
            )
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


# ---------------------------------------------------------------------------
# rank_trials: post-hoc reranking of an Optuna study output dir
# ---------------------------------------------------------------------------
# Metric -> ('min'/'max' direction, getter callable)
_RANKABLE_METRICS: dict[str, tuple[str, Callable[[dict], Optional[float]]]] = {
    'drift_m':            ('min', lambda s: s.get('drift_m')),
    'drift_per_path':     ('min', lambda s: s.get('drift_per_path')),
    'mean_icp_ratio':     ('max', lambda s: s.get('mean_icp_ratio')),
    'loop_closure_count': ('max', lambda s: s.get('loop_closure_count')),
    'n_poses':            ('max', lambda s: s.get('n_poses')),
    'path_length_m':      ('max', lambda s: s.get('path_length_m')),
    'duration_s':         ('max', lambda s: s.get('duration_s')),
}


def _bag_stats(bag_dir: Path, *, rescore: bool) -> Optional[dict]:
    """Return the stats dict for one bag run, rescoring if requested or if no
    cached metrics.json exists. Returns None if the bag run wasn't scoreable.
    """
    metrics_json = bag_dir / 'metrics.json'
    cached: Optional[dict] = None
    if metrics_json.exists() and not rescore:
        try:
            data = json.loads(metrics_json.read_text())
            if data.get('success') and data.get('stats'):
                cached = data['stats']
        except (json.JSONDecodeError, KeyError):
            cached = None
    if cached is not None:
        return cached
    db = bag_dir / 'rtabmap.db'
    if not db.exists():
        return None
    result = score_run(db, bag_dir)
    if not result.success or result.stats is None:
        return None
    return asdict(result.stats)


def rank_main() -> int:
    parser = argparse.ArgumentParser(
        description='Re-rank trials in a study output dir by any metric. '
                    'Reads cached metrics.json files, rescoring transparently '
                    'when the requested metric is missing from the cache.'
    )
    parser.add_argument('study_dir', type=Path, help='Optimizer --output-root directory.')
    parser.add_argument(
        '--metric', default='drift_per_path', choices=list(_RANKABLE_METRICS),
        help='Metric to rank by (default: drift_per_path).',
    )
    parser.add_argument('--top', type=int, default=10, help='Show top N trials (default: 10).')
    parser.add_argument(
        '--rescore', action='store_true',
        help='Force re-running score_run on every DB instead of trusting cached metrics.json.',
    )
    parser.add_argument(
        '--min-bags', type=int, default=None,
        help='Drop trials where fewer than this many bags produced a scoreable trajectory '
             '(default: require all bags).',
    )
    args = parser.parse_args()

    direction, getter = _RANKABLE_METRICS[args.metric]
    rescore = args.rescore

    trials: list[dict] = []
    bag_count_max = 0
    for trial_dir in sorted(args.study_dir.glob('trial_*')):
        if not trial_dir.is_dir():
            continue
        per_bag: list[tuple[str, float]] = []
        n_bags = 0
        for bag_dir in sorted(d for d in trial_dir.iterdir() if d.is_dir()):
            if bag_dir.name == '__pycache__':
                continue
            n_bags += 1
            stats = _bag_stats(bag_dir, rescore=rescore)
            if stats is None:
                continue
            value = getter(stats)
            if value is None:
                # Metric missing from cached stats — force a rescore for this bag.
                stats = _bag_stats(bag_dir, rescore=True)
                if stats is None:
                    continue
                value = getter(stats)
            if value is not None:
                per_bag.append((bag_dir.name, float(value)))
        bag_count_max = max(bag_count_max, n_bags)
        if per_bag:
            trial_n = int(trial_dir.name.split('_')[1])
            mean = sum(v for _, v in per_bag) / len(per_bag)
            trials.append({'n': trial_n, 'mean': mean, 'per_bag': per_bag, 'n_bags': n_bags})

    min_bags = args.min_bags if args.min_bags is not None else bag_count_max
    qualifying = [t for t in trials if len(t['per_bag']) >= min_bags]

    reverse = (direction == 'max')
    qualifying.sort(key=lambda t: t['mean'], reverse=reverse)

    print(f'Ranking by {args.metric} ({direction}); '
          f'requiring ≥{min_bags}/{bag_count_max} scoreable bags per trial.')
    print(f'Trials considered: {len(qualifying)} / {len(trials)} '
          f'(dropped {len(trials) - len(qualifying)} for incomplete bag coverage).')
    print()
    print(f'  {"rank":<5}{"trial":<7}{"mean":<14}per-bag')
    for i, t in enumerate(qualifying[:args.top]):
        bag_strs = ' '.join(f'{b}={v:.3f}' for b, v in t['per_bag'])
        print(f'  {i+1:<5}{t["n"]:<7}{t["mean"]:>10.4f}    {bag_strs}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
