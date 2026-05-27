"""Cross-verify RTAB-Map's estimated trajectory against Webots ground truth.

Reads a bag containing both /ground_truth/odom (true pose from Webots
supervisor) and the RTAB-Map estimated pose stream (defaults to /rtabmap/odom
or /rtabmap/mapPath). Aligns the two trajectories with Umeyama's SE(3) closed
form and emits standard SLAM benchmark metrics:

    ATE_rmse           position error after alignment (m, RMSE)
    ATE_mean           mean position error (m)
    ATE_max            worst-case position error (m)
    final_drift_m      euclidean distance between the last poses
    final_drift_yaw    yaw error between the last poses (rad)
    trajectory_length  ground-truth path length (m)
    drift_ratio        final_drift_m / trajectory_length

Writes a JSON report next to the bag. Pure numpy + builtin ROS 2 bag reader,
no scipy/evo dependency.

CLI:
    python -m rove_sim_webots.validator --bag <path> [--out validation.json]
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np


# ---------- Pose ----------

@dataclass
class Pose:
    t: float       # seconds (header stamp)
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


def _quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


# ---------- Bag reading ----------

def _read_bag(bag_path: Path) -> Dict[str, list]:
    """Return {topic_name: [(timestamp_ns, msg), ...]} for a rosbag2 SQLite bag."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    import rosbag2_py

    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_path), storage_id='sqlite3',
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr', output_serialization_format='cdr',
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    by_topic: Dict[str, list] = {name: [] for name in type_map}
    msg_classes = {name: get_message(typ) for name, typ in type_map.items()}

    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        msg = deserialize_message(raw, msg_classes[topic])
        by_topic[topic].append((t_ns, msg))
    return by_topic


def _odom_to_poses(msgs) -> List[Pose]:
    """Convert Odometry messages to poses, deduped + sorted by header time.

    When SimBagEvaluator records the output of `ros2 bag play`, every input
    GT message ends up captured ~2x with out-of-order receive timestamps.
    Iterating in receive-order then computes back-and-forth jumps between
    duplicates and produces huge bogus path lengths (88 km on a 250 s bag).
    Dedup by header timestamp, then sort.
    """
    seen = {}
    for _t_ns, m in msgs:
        s = m.header.stamp
        t = s.sec + s.nanosec * 1e-9
        if t in seen:
            continue
        seen[t] = Pose(
            t=t,
            x=m.pose.pose.position.x,
            y=m.pose.pose.position.y,
            z=m.pose.pose.position.z,
            qx=m.pose.pose.orientation.x,
            qy=m.pose.pose.orientation.y,
            qz=m.pose.pose.orientation.z,
            qw=m.pose.pose.orientation.w,
        )
    return [seen[t] for t in sorted(seen)]


def _path_to_poses(msgs) -> List[Pose]:
    """Use the *last* nav_msgs/Path on the topic — RTAB-Map republishes the
    full optimized path each time, so the last one is the final estimate."""
    if not msgs:
        return []
    _t_ns, last = msgs[-1]
    out: List[Pose] = []
    for ps in last.poses:
        s = ps.header.stamp
        out.append(Pose(
            t=s.sec + s.nanosec * 1e-9,
            x=ps.pose.position.x,
            y=ps.pose.position.y,
            z=ps.pose.position.z,
            qx=ps.pose.orientation.x,
            qy=ps.pose.orientation.y,
            qz=ps.pose.orientation.z,
            qw=ps.pose.orientation.w,
        ))
    return out


# ---------- Alignment + metrics ----------

def _associate(gt: List[Pose], est: List[Pose], max_dt: float = 0.05) -> List[Tuple[Pose, Pose]]:
    """Pair each estimated pose with the nearest ground-truth pose by stamp,
    rejecting pairs further than `max_dt` seconds apart. Returns sorted pairs."""
    if not gt or not est:
        return []
    gt_sorted = sorted(gt, key=lambda p: p.t)
    gt_t = np.array([p.t for p in gt_sorted])
    pairs: List[Tuple[Pose, Pose]] = []
    for e in est:
        idx = int(np.searchsorted(gt_t, e.t))
        candidates = []
        if idx > 0:
            candidates.append(idx - 1)
        if idx < len(gt_sorted):
            candidates.append(idx)
        best_i, best_dt = None, max_dt + 1.0
        for c in candidates:
            dt = abs(gt_sorted[c].t - e.t)
            if dt < best_dt:
                best_i, best_dt = c, dt
        if best_i is not None and best_dt <= max_dt:
            pairs.append((gt_sorted[best_i], e))
    return pairs


def _umeyama(src: np.ndarray, dst: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Closed-form SE(3) (no scale) alignment, rotation R + translation t
    that minimises ||R src + t - dst||^2. Inputs are (N, 3)."""
    src_mean = src.mean(axis=0)
    dst_mean = dst.mean(axis=0)
    src_c = src - src_mean
    dst_c = dst - dst_mean
    H = src_c.T @ dst_c
    U, _S, Vt = np.linalg.svd(H)
    D = np.eye(3)
    if np.linalg.det(U @ Vt) < 0:
        D[2, 2] = -1.0
    R = Vt.T @ D @ U.T
    t = dst_mean - R @ src_mean
    return R, t


@dataclass
class ValidationResult:
    n_gt_poses: int
    n_est_poses: int
    n_pairs: int
    duration_s: float
    trajectory_length_m: float

    ate_rmse_m: Optional[float] = None
    ate_mean_m: Optional[float] = None
    ate_median_m: Optional[float] = None
    ate_max_m: Optional[float] = None

    final_drift_m: Optional[float] = None
    final_drift_yaw_rad: Optional[float] = None
    drift_ratio: Optional[float] = None

    alignment_translation_m: List[float] = field(default_factory=list)
    alignment_rotation_matrix: List[List[float]] = field(default_factory=list)

    warnings: List[str] = field(default_factory=list)


def validate(
    bag_path: Path,
    gt_topic: str = '/ground_truth/odom',
    est_topic: Optional[str] = None,
    max_assoc_dt_s: float = 0.05,
) -> ValidationResult:
    """Run the comparison. `est_topic` is auto-detected if None."""
    by_topic = _read_bag(bag_path)

    if gt_topic not in by_topic or not by_topic[gt_topic]:
        raise SystemExit(f'No ground-truth messages on {gt_topic} in {bag_path}')
    gt_poses = _odom_to_poses(by_topic[gt_topic])

    # Auto-detect estimated trajectory source.
    candidates_path = ['/rtabmap/mapPath', '/rtabmap/local_path', '/rtabmap/global_path']
    candidates_odom = ['/rtabmap/odom', '/icp_odom']
    if est_topic is None:
        for c in candidates_path:
            if c in by_topic and by_topic[c]:
                est_topic = c
                break
        if est_topic is None:
            for c in candidates_odom:
                if c in by_topic and by_topic[c]:
                    est_topic = c
                    break
    if est_topic is None:
        raise SystemExit(
            'No RTAB-Map estimate topic found in bag. Tried: '
            f'{candidates_path + candidates_odom}'
        )

    if est_topic.endswith('Path') or est_topic.endswith('path'):
        est_poses = _path_to_poses(by_topic[est_topic])
    else:
        est_poses = _odom_to_poses(by_topic[est_topic])

    result = ValidationResult(
        n_gt_poses=len(gt_poses),
        n_est_poses=len(est_poses),
        n_pairs=0,
        duration_s=(gt_poses[-1].t - gt_poses[0].t) if len(gt_poses) > 1 else 0.0,
        trajectory_length_m=_path_length(gt_poses),
    )

    pairs = _associate(gt_poses, est_poses, max_assoc_dt_s)
    result.n_pairs = len(pairs)
    if len(pairs) < 3:
        result.warnings.append(
            f'Only {len(pairs)} gt/est pose pairs after time-association — '
            'cannot compute ATE. Check that bag contains both topics over the same window.'
        )
        return result

    gt_xyz = np.array([[g.x, g.y, g.z] for g, _e in pairs])
    est_xyz = np.array([[e.x, e.y, e.z] for _g, e in pairs])
    R, t = _umeyama(est_xyz, gt_xyz)
    est_aligned = (R @ est_xyz.T).T + t
    errs = np.linalg.norm(est_aligned - gt_xyz, axis=1)

    result.ate_rmse_m = float(np.sqrt(np.mean(errs ** 2)))
    result.ate_mean_m = float(np.mean(errs))
    result.ate_median_m = float(np.median(errs))
    result.ate_max_m = float(np.max(errs))
    result.alignment_translation_m = t.tolist()
    result.alignment_rotation_matrix = R.tolist()

    # Final-pose drift.
    g_last, e_last = pairs[-1]
    e_last_aligned = R @ np.array([e_last.x, e_last.y, e_last.z]) + t
    result.final_drift_m = float(np.linalg.norm(
        e_last_aligned - np.array([g_last.x, g_last.y, g_last.z])
    ))
    result.final_drift_yaw_rad = float(_angle_diff(
        _quat_to_yaw(g_last.qx, g_last.qy, g_last.qz, g_last.qw),
        _quat_to_yaw(e_last.qx, e_last.qy, e_last.qz, e_last.qw),
    ))
    if result.trajectory_length_m > 0:
        result.drift_ratio = result.final_drift_m / result.trajectory_length_m

    return result


def _angle_diff(a: float, b: float) -> float:
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


def _path_length(poses: List[Pose]) -> float:
    if len(poses) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(poses)):
        total += math.sqrt(
            (poses[i].x - poses[i - 1].x) ** 2
            + (poses[i].y - poses[i - 1].y) ** 2
            + (poses[i].z - poses[i - 1].z) ** 2
        )
    return total


def main() -> int:
    p = argparse.ArgumentParser(prog='validator')
    p.add_argument('--bag', required=True, help='Path to rosbag2 SQLite bag dir.')
    p.add_argument('--gt-topic', default='/ground_truth/odom')
    p.add_argument('--est-topic', default=None, help='Auto-detected if omitted.')
    p.add_argument('--max-assoc-dt', type=float, default=0.05,
                   help='Max time delta (s) for matching gt/est poses by timestamp.')
    p.add_argument('--out', default=None,
                   help='Output JSON path. Defaults to <bag>/validation.json.')
    args = p.parse_args()

    bag = Path(args.bag).expanduser().resolve()
    if not bag.exists():
        raise SystemExit(f'Bag not found: {bag}')

    out_path = Path(args.out) if args.out else (bag / 'validation.json')

    result = validate(
        bag_path=bag,
        gt_topic=args.gt_topic,
        est_topic=args.est_topic,
        max_assoc_dt_s=args.max_assoc_dt,
    )

    out_path.write_text(json.dumps(asdict(result), indent=2))
    print(json.dumps(asdict(result), indent=2))
    print(f'\n[validator] wrote {out_path}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
