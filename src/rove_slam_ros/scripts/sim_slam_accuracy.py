#!/usr/bin/env python3
"""Analyze SLAM-vs-GT in the Webots sim.

Inputs:
  /tmp/sim_slam_map_test/correlation_bag       — sim's /livox/lidar +
                                                  /ground_truth/odom + /tf_static
  /tmp/sim_slam_map_test/slam_replay/slam_bag  — SLAM's /odom (from replaying
                                                  the sim bag through rove_slam_node)

Outputs:
  /tmp/sim_slam_map_test/report.txt
  /tmp/sim_slam_map_test/cloud_slam.pcd    (SLAM-pose-accumulated)
  /tmp/sim_slam_map_test/cloud_gt.pcd      (GT-pose-accumulated)
  /tmp/sim_slam_map_test/cloud_slam_aligned.pcd  (after ICP-to-GT)

Method:
  1. For each lidar scan, look up SLAM's pose at scan time AND GT pose at scan time.
  2. Accumulate two clouds in their respective frames:
       cloud_slam = ∑ T_slam_baselink × T_baselink_livox × scan
       cloud_gt   = ∑ T_world_baselink × T_baselink_livox × scan
  3. Voxel-downsample both (3 cm).
  4. ICP-align cloud_slam → cloud_gt (point-to-plane). This removes any constant
     frame offset + extrinsic difference, leaving only non-rigid SLAM error.
  5. Per-point NN distance from aligned cloud_slam to cloud_gt → distribution.
  6. Also compute pose-by-pose error: for each scan time, transform-error between
     SLAM's pose and GT's pose (after a one-time initial-pose alignment).
"""

import math, sys
import numpy as np
import sqlite3
from collections import defaultdict
from pathlib import Path

import rclpy
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage

import open3d as o3d

SIM_BAG = "/tmp/sim_slam_map_test/correlation_bag/correlation_bag_0.db3"
SLAM_BAG_DIR = Path("/tmp/sim_slam_map_test/slam_replay/slam_bag")
SLAM_BAG = next(SLAM_BAG_DIR.glob("*.db3"))
OUT = Path("/tmp/sim_slam_map_test")


def read(db, topic, msg_type, rebase_t0=None):
    """Return [(t_rel, msg)]. t_rel is db-timestamp - rebase_t0 (if given),
    otherwise raw db-timestamp / 1e9. Use rebase_t0 to put two separately
    recorded bags onto a common time axis (replay was 1.0x)."""
    con = sqlite3.connect(f"file:{db}?mode=ro&immutable=1", uri=True)
    topics = {r[1]: r[0] for r in con.execute("SELECT id, name FROM topics")}
    if topic not in topics:
        return []
    rows = list(con.execute(
        "SELECT data, timestamp FROM messages WHERE topic_id=? ORDER BY timestamp",
        (topics[topic],)))
    if not rows:
        return []
    t0 = rows[0][1] / 1e9 if rebase_t0 is None else rebase_t0
    return [(ts * 1e-9 - t0, deserialize_message(d, msg_type)) for d, ts in rows]


def bag_t0(db):
    """Earliest message timestamp in the bag (seconds)."""
    con = sqlite3.connect(f"file:{db}?mode=ro&immutable=1", uri=True)
    r = con.execute("SELECT MIN(timestamp) FROM messages").fetchone()
    return r[0] / 1e9 if r and r[0] else 0.0


def pose_to_T(p):
    """nav_msgs Pose → 4x4 transform."""
    T = np.eye(4)
    q = p.orientation
    n = (q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w) ** 0.5
    qx, qy, qz, qw = q.x/n, q.y/n, q.z/n, q.w/n
    T[:3, :3] = np.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw),   1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw),   1-2*(qx*qx+qy*qy)],
    ])
    T[:3, 3] = (p.position.x, p.position.y, p.position.z)
    return T


def rpy_to_R(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    Ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
    Rz = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
    return Rz @ Ry @ Rx


def interp_pose(poses, t):
    """Linear/SLERP-ish interpolation of (t, T) list."""
    if t <= poses[0][0]:
        return poses[0][1]
    if t >= poses[-1][0]:
        return poses[-1][1]
    lo, hi = 0, len(poses) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if poses[mid][0] <= t:
            lo = mid
        else:
            hi = mid
    t0, T0 = poses[lo]; t1, T1 = poses[hi]
    if t1 == t0:
        return T0
    a = (t - t0) / (t1 - t0)
    # Linear translation, nlerp rotation (good enough for small steps)
    p = (1 - a) * T0[:3, 3] + a * T1[:3, 3]
    # Quaternion from rotation matrix
    def Rq(R):
        tr = R[0,0]+R[1,1]+R[2,2]
        if tr > 0:
            s = math.sqrt(tr + 1.0) * 2
            return np.array([(R[2,1]-R[1,2])/s, (R[0,2]-R[2,0])/s, (R[1,0]-R[0,1])/s, 0.25*s])
        i = np.argmax(np.diag(R))
        if i == 0:
            s = math.sqrt(1+R[0,0]-R[1,1]-R[2,2])*2
            return np.array([0.25*s, (R[0,1]+R[1,0])/s, (R[0,2]+R[2,0])/s, (R[2,1]-R[1,2])/s])
        if i == 1:
            s = math.sqrt(1+R[1,1]-R[0,0]-R[2,2])*2
            return np.array([(R[0,1]+R[1,0])/s, 0.25*s, (R[1,2]+R[2,1])/s, (R[0,2]-R[2,0])/s])
        s = math.sqrt(1+R[2,2]-R[0,0]-R[1,1])*2
        return np.array([(R[0,2]+R[2,0])/s, (R[1,2]+R[2,1])/s, 0.25*s, (R[1,0]-R[0,1])/s])
    q0 = Rq(T0[:3,:3]); q1 = Rq(T1[:3,:3])
    if np.dot(q0, q1) < 0: q1 = -q1
    q = (1-a)*q0 + a*q1; q /= np.linalg.norm(q)
    qx, qy, qz, qw = q
    T = np.eye(4)
    T[:3,:3] = np.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw),   1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw),   1-2*(qx*qx+qy*qy)],
    ])
    T[:3,3] = p
    return T


def decode_cloud(msg):
    """sensor_msgs/PointCloud2 → (N, 3) ndarray. Assumes float32 x,y,z fields."""
    off_x = off_y = off_z = -1
    for f in msg.fields:
        if f.name == "x": off_x = f.offset
        elif f.name == "y": off_y = f.offset
        elif f.name == "z": off_z = f.offset
    n = msg.width * msg.height
    if n == 0 or off_x < 0: return np.empty((0, 3))
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, msg.point_step)
    out = np.zeros((n, 3), dtype=np.float64)
    out[:, 0] = np.frombuffer(raw[:, off_x:off_x+4].tobytes(), dtype=np.float32)
    out[:, 1] = np.frombuffer(raw[:, off_y:off_y+4].tobytes(), dtype=np.float32)
    out[:, 2] = np.frombuffer(raw[:, off_z:off_z+4].tobytes(), dtype=np.float32)
    # Drop NaN / inf
    valid = np.isfinite(out).all(axis=1)
    out = out[valid]
    # Drop near-zero (lidar self-returns)
    r = np.linalg.norm(out, axis=1)
    out = out[(r > 0.5) & (r < 30.0)]
    return out


def main():
    print("Loading data...", file=sys.stderr)
    # Each bag was recorded with its own wall-clock origin. Replay was 1.0x,
    # so within-bag timing is preserved. Rebase each to start at 0; the two
    # streams are then time-aligned modulo bag-startup skew (typically <2s).
    sim_t0 = bag_t0(SIM_BAG)
    slam_t0 = bag_t0(str(SLAM_BAG))
    print(f"  sim bag starts: {sim_t0:.3f}", file=sys.stderr)
    print(f"  slam bag starts: {slam_t0:.3f}", file=sys.stderr)
    gt_msgs = read(SIM_BAG, "/ground_truth/odom", Odometry, rebase_t0=sim_t0)
    lidar_msgs = read(SIM_BAG, "/livox/lidar", PointCloud2, rebase_t0=sim_t0)
    slam_msgs = read(str(SLAM_BAG), "/odom", Odometry, rebase_t0=slam_t0)

    # Find SLAM's first non-identity pose — that's when the replay first
    # delivered a scan and SLAM started integrating. Re-rebase SLAM so
    # this time becomes t=0, matching sim_t=0 (first lidar scan).
    def is_origin(m, eps=1e-6):
        p = m.pose.pose
        return (abs(p.position.x) < eps and abs(p.position.y) < eps
                and abs(p.orientation.z) < eps and abs(p.orientation.w - 1) < eps)
    slam_offset_in_bag = 0.0
    for t, m in slam_msgs:
        if not is_origin(m):
            slam_offset_in_bag = t
            break
    if slam_offset_in_bag > 0.1:
        print(f"  SLAM first-motion at bag-rel t={slam_offset_in_bag:.3f}s; "
              f"re-rebasing", file=sys.stderr)
        slam_msgs = [(t - slam_offset_in_bag, m) for t, m in slam_msgs]
    print(f"  GT poses: {len(gt_msgs)}", file=sys.stderr)
    print(f"  SLAM poses: {len(slam_msgs)}", file=sys.stderr)
    print(f"  lidar scans: {len(lidar_msgs)}", file=sys.stderr)

    if not (gt_msgs and slam_msgs and lidar_msgs):
        print("Missing data — aborting"); return 1

    gt_poses = [(t, pose_to_T(m.pose.pose)) for t, m in gt_msgs]
    slam_poses = [(t, pose_to_T(m.pose.pose)) for t, m in slam_msgs]
    # Drop the warmup SLAM identity poses (anything t < 0 after rebase)
    slam_poses = [p for p in slam_poses if p[0] >= 0]

    # Sim's base→livox (from sim driver source): xyz=(-0.3, 0, 0.28) rpy=(0, 30°, 180°)
    T_base_lidar_sim = np.eye(4)
    T_base_lidar_sim[:3, :3] = rpy_to_R(0.0, math.radians(30.0), math.pi)
    T_base_lidar_sim[:3, 3] = (-0.3, 0.0, 0.28)

    # SLAM's hardcoded extrinsic (used internally when urdf_extrinsic=true):
    # xyz=(-0.30, 0, 0.318) rpy=(0, 30°, 180°). Different z by 0.038 m.
    T_base_lidar_slam = np.eye(4)
    T_base_lidar_slam[:3, :3] = rpy_to_R(0.0, math.radians(30.0), math.pi)
    T_base_lidar_slam[:3, 3] = (-0.30, 0.0, 0.318)

    # Time window: only use scans where both pose streams cover.
    t_min = max(gt_poses[0][0], slam_poses[0][0])
    t_max = min(gt_poses[-1][0], slam_poses[-1][0])
    print(f"  comparable window: [{t_min:.2f}, {t_max:.2f}] = {t_max-t_min:.1f}s",
          file=sys.stderr)

    # Per-scan pose error (after one-time initial-pose alignment)
    # Pick first usable scan time to set the initial alignment.
    usable = [(t, m) for t, m in lidar_msgs if t_min < t < t_max]
    print(f"  usable scans: {len(usable)}", file=sys.stderr)
    if not usable: return 1

    # Build accumulated clouds — both in their own reference frame.
    print("Building accumulated clouds...", file=sys.stderr)
    pts_slam = []; pts_gt = []
    n_pts_slam = n_pts_gt = 0

    pose_errors = []  # list of (t, slam_xyz, gt_xyz_in_world, dist_xy, yaw_err_rad)

    # One-time initial pose alignment for pose-error reporting:
    # find first scan time, get both poses there, compute T_slamframe_world.
    t0 = usable[0][0]
    T_slam0 = interp_pose(slam_poses, t0)
    T_gt0   = interp_pose(gt_poses, t0)
    # Align: T_align_slam_to_world = T_gt0 × T_slam0^-1
    T_align = T_gt0 @ np.linalg.inv(T_slam0)

    for t, m in usable:
        pts_local = decode_cloud(m)
        if pts_local.size == 0:
            continue
        T_slam = interp_pose(slam_poses, t)
        T_gt   = interp_pose(gt_poses, t)
        # SLAM cloud in SLAM map frame (then aligned later via ICP):
        T_slam_lidar = T_slam @ T_base_lidar_slam
        # GT cloud in sim world frame:
        T_gt_lidar = T_gt @ T_base_lidar_sim
        # Transform points (homogeneous)
        ones = np.ones((pts_local.shape[0], 1))
        Ph = np.hstack([pts_local, ones])
        pts_slam.append((T_slam_lidar @ Ph.T).T[:, :3])
        pts_gt.append((T_gt_lidar @ Ph.T).T[:, :3])
        n_pts_slam += pts_local.shape[0]
        n_pts_gt += pts_local.shape[0]

        # Pose error: SLAM's pose, transformed to GT frame via T_align, vs GT pose
        slam_in_gt = T_align @ T_slam
        dx = slam_in_gt[0, 3] - T_gt[0, 3]
        dy = slam_in_gt[1, 3] - T_gt[1, 3]
        dz = slam_in_gt[2, 3] - T_gt[2, 3]
        # Yaw error: extract yaw from rotation matrices and diff.
        def yaw_of(R):
            return math.atan2(R[1, 0], R[0, 0])
        yaw_err = yaw_of(slam_in_gt[:3,:3]) - yaw_of(T_gt[:3,:3])
        # Wrap to [-pi, pi]
        while yaw_err > math.pi: yaw_err -= 2*math.pi
        while yaw_err < -math.pi: yaw_err += 2*math.pi
        pose_errors.append((t - t0, dx, dy, dz, math.sqrt(dx*dx + dy*dy), yaw_err))

    slam_cloud = np.vstack(pts_slam)
    gt_cloud = np.vstack(pts_gt)
    print(f"  SLAM cloud raw: {slam_cloud.shape[0]} points", file=sys.stderr)
    print(f"  GT cloud raw:   {gt_cloud.shape[0]} points", file=sys.stderr)

    # Voxel downsample to make ICP fast + KDTree manageable
    def voxel(pts, voxel=0.03):
        keys = np.floor(pts / voxel).astype(np.int64)
        # Use Cantor pairing for deterministic dedup
        uniq, idx = np.unique(keys, axis=0, return_index=True)
        return pts[idx]
    slam_ds = voxel(slam_cloud, 0.03)
    gt_ds = voxel(gt_cloud, 0.03)
    print(f"  SLAM cloud after voxel 3cm: {slam_ds.shape[0]}", file=sys.stderr)
    print(f"  GT cloud after voxel 3cm:   {gt_ds.shape[0]}", file=sys.stderr)

    # Open3D point clouds for ICP
    pcd_slam = o3d.geometry.PointCloud()
    pcd_slam.points = o3d.utility.Vector3dVector(slam_ds)
    pcd_gt = o3d.geometry.PointCloud()
    pcd_gt.points = o3d.utility.Vector3dVector(gt_ds)

    # ICP point-to-point — robust + simple
    print("Running ICP slam → gt (point-to-point)...", file=sys.stderr)
    icp = o3d.pipelines.registration.registration_icp(
        pcd_slam, pcd_gt, max_correspondence_distance=0.5,
        init=T_align,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50)
    )
    T_icp = icp.transformation
    print(f"  ICP fitness: {icp.fitness:.3f}, inlier RMSE: {icp.inlier_rmse:.4f}",
          file=sys.stderr)
    pcd_slam_aligned = pcd_slam.transform(T_icp.copy())

    # Per-point NN distance: each aligned SLAM point → nearest GT point
    print("Computing NN distances...", file=sys.stderr)
    tree = o3d.geometry.KDTreeFlann(pcd_gt)
    dists = np.empty(len(pcd_slam_aligned.points))
    for i, p in enumerate(pcd_slam_aligned.points):
        _, idx, sqd = tree.search_knn_vector_3d(p, 1)
        dists[i] = math.sqrt(sqd[0])

    # Also reverse: each GT point → nearest aligned SLAM point
    tree_s = o3d.geometry.KDTreeFlann(pcd_slam_aligned)
    dists_rev = np.empty(len(pcd_gt.points))
    for i, p in enumerate(pcd_gt.points):
        _, idx, sqd = tree_s.search_knn_vector_3d(p, 1)
        dists_rev[i] = math.sqrt(sqd[0])

    # Save clouds
    o3d.io.write_point_cloud(str(OUT / "cloud_slam.pcd"), pcd_slam, compressed=True)
    o3d.io.write_point_cloud(str(OUT / "cloud_gt.pcd"), pcd_gt, compressed=True)
    o3d.io.write_point_cloud(str(OUT / "cloud_slam_aligned.pcd"), pcd_slam_aligned,
                              compressed=True)

    # Write report
    pose_errors = np.array(pose_errors)
    report = OUT / "report.txt"
    with open(report, "w") as f:
        f.write("=== SLAM-vs-GT accuracy report — Webots indoor_office, fast mode ===\n\n")
        f.write(f"Comparable time window: {t_max-t_min:.1f}s, "
                 f"{len(usable)} lidar scans (~{len(usable)/(t_max-t_min):.1f} Hz)\n\n")

        f.write("--- POSE ERROR (SLAM trajectory vs GT trajectory) ---\n")
        f.write("After one-time initial-pose alignment at t=0:\n")
        if len(pose_errors):
            xy = pose_errors[:, 4]
            dz = pose_errors[:, 3]
            yaw = pose_errors[:, 5]
            f.write(f"  XY error    : mean {xy.mean()*100:.2f} cm, "
                     f"median {np.median(xy)*100:.2f} cm, "
                     f"p95 {np.percentile(xy,95)*100:.2f} cm, "
                     f"max {xy.max()*100:.2f} cm\n")
            f.write(f"  Z error     : mean {dz.mean()*100:+.2f} cm, "
                     f"median {np.median(dz)*100:+.2f} cm, "
                     f"max-abs {np.abs(dz).max()*100:.2f} cm\n")
            f.write(f"  Yaw error   : mean {math.degrees(yaw.mean()):+.3f}°, "
                     f"max-abs {math.degrees(np.abs(yaw).max()):.3f}°\n")
            f.write(f"  Final XY    : {xy[-1]*100:.2f} cm  "
                     f"(after {pose_errors[-1,0]:.1f}s of motion)\n")
            f.write(f"  Final yaw   : {math.degrees(yaw[-1]):+.3f}°\n")

        f.write("\n--- MAP-CLOUD ACCURACY (point-to-point NN after ICP) ---\n")
        f.write(f"Pre-ICP rigid alignment used: initial-pose offset\n")
        f.write(f"ICP refinement: fitness {icp.fitness:.3f}, "
                 f"inlier RMSE {icp.inlier_rmse*100:.2f} cm\n\n")
        f.write(f"SLAM cloud: {len(pcd_slam_aligned.points)} pts (voxel 3 cm)\n")
        f.write(f"GT   cloud: {len(pcd_gt.points)} pts (voxel 3 cm)\n\n")
        f.write("Per-point NN distance (SLAM → GT):\n")
        f.write(f"  mean   : {dists.mean()*100:6.2f} cm\n")
        f.write(f"  median : {np.median(dists)*100:6.2f} cm\n")
        f.write(f"  p90    : {np.percentile(dists, 90)*100:6.2f} cm\n")
        f.write(f"  p95    : {np.percentile(dists, 95)*100:6.2f} cm\n")
        f.write(f"  p99    : {np.percentile(dists, 99)*100:6.2f} cm\n")
        f.write(f"  max    : {dists.max()*100:6.2f} cm\n")
        f.write("\nPer-point NN distance (GT → SLAM, complementary):\n")
        f.write(f"  mean   : {dists_rev.mean()*100:6.2f} cm\n")
        f.write(f"  median : {np.median(dists_rev)*100:6.2f} cm\n")
        f.write(f"  p95    : {np.percentile(dists_rev, 95)*100:6.2f} cm\n")
        f.write(f"  max    : {dists_rev.max()*100:6.2f} cm\n")

        f.write("\n--- DRIFT PROFILE (every ~2 s) ---\n")
        f.write("  t(s) |  XY err(cm) |  Z err(cm) |  yaw err(deg)\n")
        for i in range(0, len(pose_errors), max(1, len(pose_errors)//12)):
            t, dx, dy, dz, xy, yaw = pose_errors[i]
            f.write(f"  {t:5.1f} | {xy*100:10.2f} | {dz*100:+9.2f} | {math.degrees(yaw):+13.2f}\n")
        # last sample too
        t, dx, dy, dz, xy, yaw = pose_errors[-1]
        f.write(f"  {t:5.1f} | {xy*100:10.2f} | {dz*100:+9.2f} | {math.degrees(yaw):+13.2f}  (last)\n")

        f.write("\n--- CAVEATS ---\n")
        f.write("* SLAM ran with urdf_extrinsic=True, which uses a hardcoded\n")
        f.write("  base→lidar of xyz=(-0.30, 0, 0.318), rpy=(0, 30°, 180°). The\n")
        f.write("  sim's actual base→lidar is xyz=(-0.30, 0, 0.28), rpy=(0, 30°, 180°)\n")
        f.write("  — a 3.8 cm Z offset that ICP partially absorbs as rigid offset.\n")
        f.write("* The Webots lidar runs at ~4 Hz under fast mode here (vs the\n")
        f.write("  real MID-360's 10 Hz). Fewer scans per metre means lower\n")
        f.write("  map density and potentially noisier ICP.\n")
        f.write("* No loop closure in the trajectory (single rotation + short\n")
        f.write("  forward leg). Real-world long-loop drift is NOT exercised by\n")
        f.write("  this test.\n")
        f.write("* GT vs lidar-noise floor: even a perfect SLAM has 2-3 cm of\n")
        f.write("  per-point noise from the sim's lidar plugin, which dominates\n")
        f.write("  the residual after ICP for stationary regions.\n")

    print(f"Report written to {report}", file=sys.stderr)
    print()
    print(open(report).read())
    return 0


if __name__ == "__main__":
    rclpy.init()
    try:
        sys.exit(main())
    finally:
        rclpy.shutdown()
