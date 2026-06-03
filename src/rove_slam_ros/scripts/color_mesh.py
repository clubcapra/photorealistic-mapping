#!/usr/bin/env python3
"""Offline color-mapping tool: given a mesh + a rosbag2 with camera images +
a SLAM trajectory + a URDF, project each mesh vertex into the best-matching
camera image and write a per-vertex RGB PLY.

  color_mesh.py \
      --mesh   mesh.ply                            \
      --bag    /home/iliana/bags/rosbag2_test_camera_lidars  \
      --traj   trajectory.tum                      \
      --urdf   src/rove_description/urdf/rove_standard.urdf  \
      --intrinsics src/rove_slam_ros/config/cam_intrinsics.yaml \
      --out    colored.ply

Algorithm:
  1. Parse URDF → static T_root_camopt for each cam_optical_frame.
     The URDF root is assumed to be base_link (set --base-link-name if not).
  2. Load TUM trajectory   → ordered list[(t_ns, T_map_base)].
  3. Read all Image msgs   → per-cam list[(t_ns, image_bgr)].
  4. For each cam, pick keyframes (one per `--keyframe-stride` seconds)
     and pre-compute T_map_camopt at the image's timestamp by
     interpolating the trajectory.
  5. For each mesh vertex V (in map frame):
       For each cam keyframe:
         p_cam = (T_map_camopt)^-1 @ V
         if p_cam.z < min_depth or > max_depth: skip
         pixel = K @ (p_cam / p_cam.z)
         if pixel outside image: skip
         sample with bilinear interp
         weight = 1 / (depth^2 + eps)
       weighted-average sampled colors → vertex RGB
     Vertices with no valid sample → kept at the mesh's original color
     (gray if input had none).
  6. Write PLY with float xyz + uchar rgb.

Designed to be runnable headless: pure numpy/cv2/open3d + sqlite3, no ROS
runtime dependency. Reads the .db3 directly so it works without sourcing
the workspace.

Intrinsics YAML (one entry per cam_optical_frame name):

  cam_north_optical_frame:
    width: 640
    height: 480
    fx: 320.0
    fy: 320.0
    cx: 320.0
    cy: 240.0
    distortion_model: plumb_bob       # or "equidistant" / "none"
    D: [0.0, 0.0, 0.0, 0.0, 0.0]      # k1, k2, p1, p2, k3 for plumb_bob
  cam_east_optical_frame:
    ...

The accompanying ``cam_intrinsics.yaml`` ships PLACEHOLDER values
(HFOV=90°) — colors WILL be geometrically wrong until you replace them
with the calibration result.
"""

from __future__ import annotations

import argparse
import math
import os
import sqlite3
import struct
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np
import yaml

# Trajectory + URDF parsing  ---------------------------------------------


def _rpy_to_R(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy_, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy_, -sy, 0], [sy, cy_, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _quat_to_R(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz),     2 * (qx * qy - qz * qw),     2 * (qx * qz + qy * qw)],
        [    2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz),     2 * (qy * qz - qx * qw)],
        [    2 * (qx * qz - qy * qw),     2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


def parse_urdf_chain(urdf_path: Path) -> dict[str, tuple[str, np.ndarray]]:
    """Return {child_link: (parent_link, T_parent_child)} for every joint.

    Revolute joints are treated as fixed at zero (they are all `joint_revolute_*`
    in the recovered URDF but kinematically static in this build).
    """
    root = ET.parse(str(urdf_path)).getroot()
    chain: dict[str, tuple[str, np.ndarray]] = {}
    for j in root.findall("joint"):
        parent = j.find("parent").get("link")
        child = j.find("child").get("link")
        origin = j.find("origin")
        xyz = [0.0, 0.0, 0.0]
        rpy = [0.0, 0.0, 0.0]
        if origin is not None:
            if origin.get("xyz"):
                xyz = list(map(float, origin.get("xyz").split()))
            if origin.get("rpy"):
                rpy = list(map(float, origin.get("rpy").split()))
        T = np.eye(4)
        T[:3, :3] = _rpy_to_R(*rpy)
        T[:3, 3] = xyz
        chain[child] = (parent, T)
    return chain


def compose_to_root(chain: dict[str, tuple[str, np.ndarray]],
                    leaf: str, root: str) -> np.ndarray:
    """T_root_leaf by walking up the chain."""
    T = np.eye(4)
    cur = leaf
    seen = set()
    while cur != root:
        if cur in seen:
            raise RuntimeError(f"cycle in URDF chain at {cur}")
        seen.add(cur)
        if cur not in chain:
            raise RuntimeError(f"link {cur} has no parent; cannot reach {root}")
        parent, T_parent_child = chain[cur]
        T = T_parent_child @ T  # T_parent_leaf = T_parent_child @ T_child_leaf
        cur = parent
    return T


# TUM trajectory --------------------------------------------------------


def load_tum(path: Path) -> list[tuple[int, np.ndarray]]:
    """Return [(t_ns, T_map_base)] sorted by time."""
    out = []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) < 8:
            continue
        t_s = float(parts[0])
        x, y, z = map(float, parts[1:4])
        qx, qy, qz, qw = map(float, parts[4:8])
        T = np.eye(4)
        T[:3, :3] = _quat_to_R(qx, qy, qz, qw)
        T[:3, 3] = (x, y, z)
        out.append((int(t_s * 1e9), T))
    out.sort(key=lambda r: r[0])
    return out


def interp_pose(traj: list[tuple[int, np.ndarray]], t_ns: int) -> np.ndarray | None:
    """Linear interp (position) + slerp (rotation) of T_map_base at t_ns.

    Returns None if t_ns is outside the trajectory's time bounds.
    """
    if not traj:
        return None
    if t_ns <= traj[0][0]:
        return traj[0][1] if abs(t_ns - traj[0][0]) < int(0.5e9) else None
    if t_ns >= traj[-1][0]:
        return traj[-1][1] if abs(t_ns - traj[-1][0]) < int(0.5e9) else None
    # Binary search
    lo, hi = 0, len(traj) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if traj[mid][0] <= t_ns:
            lo = mid
        else:
            hi = mid
    t0, T0 = traj[lo]
    t1, T1 = traj[hi]
    if t1 == t0:
        return T0
    a = (t_ns - t0) / (t1 - t0)
    # Linear interp of translation
    p = (1 - a) * T0[:3, 3] + a * T1[:3, 3]
    # Slerp-ish via quaternion (small steps → lerp+normalize is fine)
    R0, R1 = T0[:3, :3], T1[:3, :3]
    q0 = _R_to_quat(R0)
    q1 = _R_to_quat(R1)
    if np.dot(q0, q1) < 0:
        q1 = -q1
    q = (1 - a) * q0 + a * q1
    q /= np.linalg.norm(q)
    T = np.eye(4)
    T[:3, :3] = _quat_to_R(*q[1:], q[0])  # _quat_to_R expects (qx, qy, qz, qw)
    T[:3, 3] = p
    return T


def _R_to_quat(R: np.ndarray) -> np.ndarray:
    """(qw, qx, qy, qz) for ordering matching the interp above."""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    else:
        i = np.argmax(np.diag(R))
        if i == 0:
            s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif i == 1:
            s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s
    return np.array([qw, qx, qy, qz])


# Bag reading ----------------------------------------------------------


def find_db3(bag_dir: Path) -> Path:
    candidates = sorted(bag_dir.glob("*.db3"))
    if not candidates:
        sys.exit(f"no .db3 found in {bag_dir}")
    return candidates[0]


def list_camera_topics(db: Path) -> list[tuple[int, str, str]]:
    con = sqlite3.connect(f"file:{db}?mode=ro&immutable=1", uri=True)
    rows = list(con.execute(
        "SELECT id, name, type FROM topics WHERE type='sensor_msgs/msg/Image'"))
    con.close()
    return rows


def cam_frame_id_from_topic(topic: str) -> str:
    """`/cam_north/image_raw` → `cam_north_optical_frame` (heuristic)."""
    leaf = topic.strip("/").split("/")[0]  # `cam_north`
    return f"{leaf}_optical_frame"


def parse_image_msg(blob: bytes) -> tuple[int, str, np.ndarray] | None:
    """Decode sensor_msgs/Image. Returns (t_ns, frame_id, BGR image)."""
    try:
        pos = 4  # CDR encapsulation
        sec, nsec = struct.unpack_from("<II", blob, pos); pos += 8
        flen = struct.unpack_from("<I", blob, pos)[0]; pos += 4
        fid = blob[pos:pos + flen].rstrip(b"\x00").decode(); pos += flen
        while pos % 4: pos += 1
        height, width = struct.unpack_from("<II", blob, pos); pos += 8
        elen = struct.unpack_from("<I", blob, pos)[0]; pos += 4
        enc = blob[pos:pos + elen].rstrip(b"\x00").decode(); pos += elen
        # is_bigendian is uint8 with no alignment requirement — it sits
        # directly after the encoding string. Padding is BEFORE step (u32).
        big = blob[pos]; pos += 1
        while pos % 4: pos += 1
        step = struct.unpack_from("<I", blob, pos)[0]; pos += 4
        dlen = struct.unpack_from("<I", blob, pos)[0]; pos += 4
        pixels = np.frombuffer(blob, dtype=np.uint8, count=dlen, offset=pos)
        if enc == "rgb8":
            img = pixels.reshape(height, width, 3)[:, :, ::-1].copy()  # → BGR
        elif enc == "bgr8":
            img = pixels.reshape(height, width, 3).copy()
        elif enc == "mono8":
            img = pixels.reshape(height, width)
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        else:
            print(f"  skip frame: unsupported encoding {enc}")
            return None
        t_ns = sec * 1_000_000_000 + nsec
        return t_ns, fid, img
    except Exception as e:
        print(f"  parse_image_msg error: {e}")
        return None


def first_lidar_timestamp(db: Path) -> int | None:
    """Header-stamp ns of the bag's first lidar scan. Used to align bag
    images to a trajectory written in a different clock (e.g. wall-time
    bag replay vs bag-time images)."""
    con = sqlite3.connect(f"file:{db}?mode=ro&immutable=1", uri=True)
    row = con.execute("""
        SELECT m.data FROM messages m JOIN topics t ON m.topic_id=t.id
        WHERE t.type='sensor_msgs/msg/PointCloud2'
        ORDER BY m.timestamp LIMIT 1
    """).fetchone()
    con.close()
    if row is None:
        return None
    data = row[0]
    pos = 4
    sec, nsec = struct.unpack_from("<II", data, pos)
    return sec * 1_000_000_000 + nsec


def load_camera_keyframes(
    db: Path,
    cam_topics: list[str],
    keyframe_stride_s: float,
) -> dict[str, list[tuple[int, np.ndarray]]]:
    """For each cam topic, decode messages spaced at least `stride` apart."""
    con = sqlite3.connect(f"file:{db}?mode=ro&immutable=1", uri=True)
    topic_ids = {
        row[1]: row[0]
        for row in con.execute("SELECT id, name FROM topics") if row[1] in cam_topics
    }
    out: dict[str, list[tuple[int, np.ndarray]]] = {t: [] for t in cam_topics}
    stride_ns = int(keyframe_stride_s * 1e9)
    for topic, tid in topic_ids.items():
        last_t = 0
        kept = 0
        total = 0
        for (data, ts) in con.execute(
            "SELECT data, timestamp FROM messages WHERE topic_id=? ORDER BY timestamp",
            (tid,)
        ):
            total += 1
            if ts - last_t < stride_ns:
                continue
            parsed = parse_image_msg(data)
            if parsed is None:
                continue
            t_ns, fid, img = parsed
            out[topic].append((t_ns, img))
            last_t = ts
            kept += 1
        print(f"  {topic:30s}: kept {kept}/{total} frames "
              f"@ stride {keyframe_stride_s}s, resolution={img.shape[:2] if kept else '?'}")
    con.close()
    return out


# Mesh I/O -------------------------------------------------------------


def read_mesh(path: Path):
    import open3d as o3d
    m = o3d.io.read_triangle_mesh(str(path))
    if not m.has_vertices():
        sys.exit(f"mesh has no vertices: {path}")
    V = np.asarray(m.vertices, dtype=np.float64)
    F = np.asarray(m.triangles)
    return m, V, F


def write_colored_mesh(path: Path, m, V: np.ndarray, F: np.ndarray, colors: np.ndarray):
    import open3d as o3d
    m2 = o3d.geometry.TriangleMesh()
    m2.vertices = o3d.utility.Vector3dVector(V)
    m2.triangles = o3d.utility.Vector3iVector(F)
    m2.vertex_colors = o3d.utility.Vector3dVector(np.clip(colors / 255.0, 0, 1))
    o3d.io.write_triangle_mesh(str(path), m2, write_ascii=False)


# Projection -----------------------------------------------------------


def project_and_sample(
    Vmap: np.ndarray,                 # (N,3) map-frame
    T_map_camopt: np.ndarray,         # 4x4
    K: np.ndarray, D: np.ndarray, dmodel: str,
    image: np.ndarray,                # H x W x 3 BGR
    min_depth: float, max_depth: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Returns (color (N,3) uint8, weight (N,) float). Weight=0 where invalid."""
    H, W = image.shape[:2]
    # Convert Vmap into camera optical frame
    T_camopt_map = np.linalg.inv(T_map_camopt)
    Vh = np.hstack([Vmap, np.ones((Vmap.shape[0], 1))])  # (N,4)
    Pc = (T_camopt_map @ Vh.T).T[:, :3]                  # (N,3)
    z = Pc[:, 2]
    valid = (z > min_depth) & (z < max_depth)
    if not np.any(valid):
        return np.zeros((Vmap.shape[0], 3), dtype=np.uint8), np.zeros(Vmap.shape[0])
    # Project (optionally distort)
    pts = Pc[valid]
    if dmodel == "plumb_bob" and np.any(D != 0):
        # Use OpenCV
        rvec = np.zeros(3); tvec = np.zeros(3)
        img_pts, _ = cv2.projectPoints(pts.reshape(-1, 1, 3), rvec, tvec, K, D)
        uv = img_pts.reshape(-1, 2)
    else:
        uv = (pts[:, :2] / pts[:, 2:3]) * np.array([K[0, 0], K[1, 1]]) + np.array([K[0, 2], K[1, 2]])
    u, v = uv[:, 0], uv[:, 1]
    in_img = (u >= 0) & (u < W - 1) & (v >= 0) & (v < H - 1)
    # Bilinear sample
    u_in, v_in = u[in_img], v[in_img]
    u0 = np.floor(u_in).astype(int); v0 = np.floor(v_in).astype(int)
    du = (u_in - u0)[:, None]; dv = (v_in - v0)[:, None]
    c00 = image[v0,     u0    ].astype(np.float32)
    c10 = image[v0,     u0 + 1].astype(np.float32)
    c01 = image[v0 + 1, u0    ].astype(np.float32)
    c11 = image[v0 + 1, u0 + 1].astype(np.float32)
    c = (1 - du) * (1 - dv) * c00 + du * (1 - dv) * c10 + \
        (1 - du) * dv * c01 + du * dv * c11
    # Build full-size outputs
    color = np.zeros((Vmap.shape[0], 3), dtype=np.uint8)
    weight = np.zeros(Vmap.shape[0])
    valid_idx = np.flatnonzero(valid)
    in_img_idx = valid_idx[in_img]
    color[in_img_idx] = c.astype(np.uint8)
    z_in = z[valid_idx][in_img]
    weight[in_img_idx] = 1.0 / (z_in * z_in + 1e-3)  # closer cam → higher weight
    return color, weight


# Intrinsics YAML -----------------------------------------------------


def load_intrinsics(path: Path | None, cam_names: list[str]) -> dict[str, dict]:
    """Load intrinsics; fall back to FAKE 90°-HFOV defaults for any missing."""
    spec: dict[str, dict] = {}
    if path and path.exists():
        spec = yaml.safe_load(path.read_text()) or {}
    out: dict[str, dict] = {}
    for cam in cam_names:
        s = spec.get(cam, {})
        W = int(s.get("width", 640))
        H = int(s.get("height", 480))
        # 90° HFOV → fx = (W/2) / tan(45°) = W/2
        fx = float(s.get("fx", W / 2.0))
        fy = float(s.get("fy", fx))
        cx = float(s.get("cx", W / 2.0))
        cy = float(s.get("cy", H / 2.0))
        K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
        D = np.array(s.get("D", [0.0, 0.0, 0.0, 0.0, 0.0]), dtype=np.float64)
        out[cam] = {
            "K": K, "D": D,
            "width": W, "height": H,
            "distortion_model": s.get("distortion_model", "none"),
            "is_placeholder": cam not in spec,
        }
    return out


# Driver --------------------------------------------------------------


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mesh", type=Path, required=True)
    ap.add_argument("--bag",  type=Path, required=True, help="rosbag2 dir")
    ap.add_argument("--traj", type=Path, required=True, help="TUM trajectory")
    ap.add_argument("--urdf", type=Path, required=True)
    ap.add_argument("--intrinsics", type=Path, default=None)
    ap.add_argument("--out",  type=Path, required=True)
    ap.add_argument("--base-link-name", default="Core",
                    help="URDF link to treat as base_link (default Core, "
                         "matching rove_standard.urdf).")
    ap.add_argument("--keyframe-stride", type=float, default=0.5,
                    help="Use 1 frame per cam per this many seconds.")
    ap.add_argument("--time-align", choices=("auto", "off"), default="auto",
                    help="When 'auto' (default), compute the wall-vs-bag clock "
                         "offset from the bag's first lidar stamp vs the "
                         "trajectory's first stamp and apply it to image "
                         "timestamps. Use 'off' if your trajectory was recorded "
                         "with use_sim_time:=true (bag-time native).")
    ap.add_argument("--min-depth", type=float, default=0.20)
    ap.add_argument("--max-depth", type=float, default=15.0)
    ap.add_argument("--max-vertices", type=int, default=0,
                    help="If >0, subsample mesh vertices uniformly for speed.")
    args = ap.parse_args()

    t0 = time.time()

    # 1. URDF: compose cam_optical → base_link for every cam_*_optical_frame link
    chain = parse_urdf_chain(args.urdf)
    cam_links = sorted(c for c in chain.keys() if c.endswith("_optical_frame"))
    print(f"URDF has {len(cam_links)} *_optical_frame links: {cam_links}")
    T_base_camopt: dict[str, np.ndarray] = {}
    for cam in cam_links:
        try:
            T_base_camopt[cam] = compose_to_root(chain, cam, args.base_link_name)
            t = T_base_camopt[cam][:3, 3]
            print(f"  {cam:35s} pos in {args.base_link_name}: "
                  f"({t[0]:+.3f}, {t[1]:+.3f}, {t[2]:+.3f})")
        except RuntimeError as e:
            print(f"  {cam}: chain compose failed ({e}); skipping")

    # 2. Trajectory
    traj = load_tum(args.traj)
    if not traj:
        sys.exit("trajectory empty")
    print(f"loaded {len(traj)} trajectory poses, t spans "
          f"{(traj[-1][0] - traj[0][0]) / 1e9:.1f}s")

    # 3. Bag → camera keyframes
    db = find_db3(args.bag)
    img_topics = [r[1] for r in list_camera_topics(db)]
    print(f"bag has {len(img_topics)} image topics: {img_topics}")
    frames = load_camera_keyframes(db, img_topics, args.keyframe_stride)

    # 3b. Compute wall-time-vs-bag offset so image timestamps land in
    # the trajectory's clock domain.
    time_offset_ns = 0
    if args.time_align == "auto":
        bag_t0 = first_lidar_timestamp(db)
        traj_t0 = traj[0][0]
        if bag_t0 is None:
            print("  no lidar msg in bag — skipping time alignment")
        else:
            time_offset_ns = traj_t0 - bag_t0
            print(f"  bag lidar t0  : {bag_t0 / 1e9:.3f}s")
            print(f"  traj    t0    : {traj_t0 / 1e9:.3f}s")
            print(f"  → offset      : {time_offset_ns / 1e9:+.3f}s "
                  f"(added to image stamps)")
    # Map topic → optical frame name
    topic_to_camopt = {t: cam_frame_id_from_topic(t) for t in img_topics}

    # 4. Intrinsics
    cam_names_needed = sorted(set(topic_to_camopt.values()) & set(T_base_camopt.keys()))
    if not cam_names_needed:
        sys.exit("no cameras both in URDF and bag — check naming convention")
    K_per_cam = load_intrinsics(args.intrinsics, cam_names_needed)
    for cam, spec in K_per_cam.items():
        tag = "PLACEHOLDER" if spec["is_placeholder"] else "calibrated"
        K = spec["K"]
        print(f"  {cam:35s} {tag:11s} {spec['width']}x{spec['height']} "
              f"fx={K[0,0]:.1f} fy={K[1,1]:.1f} cx={K[0,2]:.1f} cy={K[1,2]:.1f}")

    # 5. Mesh
    m, V, F = read_mesh(args.mesh)
    print(f"loaded mesh: {len(V)} verts, {len(F)} tris")
    if args.max_vertices and len(V) > args.max_vertices:
        idx = np.random.default_rng(0).choice(len(V), args.max_vertices, replace=False)
        V_sub = V[idx]
        print(f"  subsampling to {args.max_vertices} vertices for speed")
        # we'll just write color per ORIGINAL vertex; subsample only colors used as KD
        # — but to keep things straightforward, color the full mesh.
    # Run full set.

    # 6. Project + accumulate
    color_acc = np.zeros((len(V), 3), dtype=np.float64)
    weight_acc = np.zeros(len(V), dtype=np.float64)
    sample_count = 0
    skip_count = 0
    for topic, kf in frames.items():
        cam = topic_to_camopt[topic]
        if cam not in T_base_camopt:
            print(f"  {topic}: no URDF chain to {args.base_link_name}, skip")
            continue
        if not kf:
            print(f"  {topic}: no frames, skip")
            continue
        spec = K_per_cam[cam]
        K = spec["K"]; D = spec["D"]; dmodel = spec["distortion_model"]
        for (t_ns, img) in kf:
            t_ns_aligned = t_ns + time_offset_ns
            T_map_base = interp_pose(traj, t_ns_aligned)
            if T_map_base is None:
                skip_count += 1
                continue
            T_map_camopt = T_map_base @ T_base_camopt[cam]
            c, w = project_and_sample(V, T_map_camopt, K, D, dmodel, img,
                                      args.min_depth, args.max_depth)
            color_acc += c.astype(np.float64) * w[:, None]
            weight_acc += w
            sample_count += int((w > 0).sum())

    # 7. Finalize colors
    good = weight_acc > 0
    colors = np.full((len(V), 3), 128, dtype=np.uint8)
    if np.any(good):
        colors[good] = np.clip(
            color_acc[good] / weight_acc[good, None], 0, 255
        ).astype(np.uint8)
    pct = 100.0 * good.sum() / len(V)
    print(f"\n--- colored {good.sum()}/{len(V)} verts ({pct:.1f}%); "
          f"{sample_count} samples total; {skip_count} frames out-of-traj ---")

    # 8. Write
    write_colored_mesh(args.out, m, V, F, colors)
    elapsed = time.time() - t0
    print(f"wrote {args.out}  ({elapsed:.1f}s wall-clock)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
