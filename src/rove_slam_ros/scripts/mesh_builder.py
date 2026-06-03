#!/usr/bin/env python3
"""Mesh-builder ROS 2 node.

Drives the offline mesh tools (`tools/build_mesh.py` in the rove_slam
submodule) from inside the running ROS graph, selectable at launch time
via the `method` parameter:

    method := poisson | bpa | tsdf | nvblox

The node accumulates the live SLAM trajectory from /odom and (when a
.rec recording_dir is not provided) the raw scans from /livox/lidar.
A `~build_mesh` Trigger service kicks off the build at any point. With
`build_on_shutdown:=true` (default), the node also auto-builds when the
process is told to exit.

Outputs land in `output_dir/`:
    trajectory.tum         live-recorded SLAM trajectory
    dense.pcd              dense world cloud (poisson/bpa intermediate)
    mesh_<method>.ply      reconstructed mesh
    build.log              tool stdout/stderr for that build

Parameters (with defaults):
    method                 'poisson'
    output_dir             '/tmp/rove_slam_mesh'
    recording_dir          ''          # set when bag-replaying; else
                                       # node writes scans to a temp .rec
    voxel                  0.05        # m, applies to all methods
    poisson_depth          9
    poisson_density_q      0.05
    tsdf_trunc             0.20
    urdf_extrinsic         True
    build_on_shutdown      True

Service:
    ~build_mesh   std_srvs/Trigger   → on success: response.message holds
                                       the saved mesh path
"""

from __future__ import annotations

import os
import signal
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger


# Wire format constants (must match the .rec spec — see
# rove_slam/docs/wire-format.md).
MSG_LIDAR = 0x01
WIRE_HEADER_FMT = "<IHHIQ"
WIRE_HEADER_SIZE = 20


def _pc2_to_xyzi(msg: PointCloud2) -> np.ndarray:
    """Decode PointCloud2 to (N, 4) float32 of (x, y, z, intensity).
    Intensity 0 when not present."""
    off_x = off_y = off_z = off_i = -1
    for fd in msg.fields:
        if fd.name == "x": off_x = fd.offset
        elif fd.name == "y": off_y = fd.offset
        elif fd.name == "z": off_z = fd.offset
        elif fd.name == "intensity": off_i = fd.offset
    if off_x < 0 or off_y < 0 or off_z < 0:
        return np.empty((0, 4), dtype=np.float32)
    n = msg.width * msg.height
    if n == 0:
        return np.empty((0, 4), dtype=np.float32)
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, msg.point_step)
    out = np.zeros((n, 4), dtype=np.float32)
    out[:, 0] = np.frombuffer(raw[:, off_x:off_x + 4].tobytes(), dtype=np.float32)
    out[:, 1] = np.frombuffer(raw[:, off_y:off_y + 4].tobytes(), dtype=np.float32)
    out[:, 2] = np.frombuffer(raw[:, off_z:off_z + 4].tobytes(), dtype=np.float32)
    if off_i >= 0:
        out[:, 3] = np.frombuffer(raw[:, off_i:off_i + 4].tobytes(), dtype=np.float32)
    return out


def _write_lidar_recording(out_dir: Path,
                           scans: list[tuple[int, np.ndarray]]) -> Path:
    """Write a minimal .rec-format payload that `tsdf_mesh.py` and
    `build_dense_map.py` can consume. Lidar-only, no per-point time."""
    out_dir.mkdir(parents=True, exist_ok=True)
    payload_path = out_dir / "payload.bin"
    index_path = out_dir / "index.bin"
    payload_path.parent.mkdir(parents=True, exist_ok=True)
    with open(payload_path, "wb") as f, open(index_path, "wb") as idx:
        for ts_ns, xyzi in scans:
            n = xyzi.shape[0]
            body = struct.pack("<IB", n, 0)
            body += b"\x00\x00\x00"               # padding to 8-byte alignment
            body += xyzi.tobytes()                # 16 bytes per point (xyzi)
            header = struct.pack(WIRE_HEADER_FMT, 0, 0, MSG_LIDAR,
                                  len(body), ts_ns)
            f.write(header)
            f.write(body)
            # 21-byte index entry: type(1) + ts(8) + offset(8) + size(4)
            idx.write(struct.pack("<BQQI", MSG_LIDAR, ts_ns, 0, len(body)))
    return out_dir


class MeshBuilder(Node):
    def __init__(self) -> None:
        super().__init__("mesh_builder")

        self.method = self.declare_parameter("method", "poisson").value
        self.output_dir = Path(
            self.declare_parameter("output_dir", "/tmp/rove_slam_mesh").value
        )
        self.recording_dir = self.declare_parameter("recording_dir", "").value
        self.voxel = float(self.declare_parameter("voxel", 0.05).value)
        self.poisson_depth = int(self.declare_parameter("poisson_depth", 9).value)
        self.poisson_dq = float(
            self.declare_parameter("poisson_density_q", 0.05).value
        )
        self.tsdf_trunc = float(self.declare_parameter("tsdf_trunc", 0.20).value)
        self.urdf_extrinsic = bool(
            self.declare_parameter("urdf_extrinsic", True).value
        )
        self.build_on_shutdown = bool(
            self.declare_parameter("build_on_shutdown", True).value
        )
        self.lidar_topic = self.declare_parameter("lidar_topic", "/livox/lidar").value
        self.odom_topic = self.declare_parameter("odom_topic", "/odom").value
        # Path to the rove_slam submodule's tools/build_mesh.py — resolved
        # relative to the install share dir of this package by the launch.
        # Override with absolute path if running outside ament.
        default_tool = (
            "/home/iliana/prog/photorealistic-mapping/src/rove_slam_ros/"
            "external/rove_slam/tools/build_mesh.py"
        )
        self.build_mesh_tool = Path(
            self.declare_parameter("build_mesh_tool", default_tool).value
        )
        # Color-mesh hook. When colorize=true and bag_path is set, after a
        # mesh build we invoke color_mesh.py to project camera images from
        # the bag onto the mesh and write mesh_<method>_colored.ply.
        self.colorize = bool(self.declare_parameter("colorize", False).value)
        self.bag_path = self.declare_parameter("bag_path", "").value
        self.urdf_path = self.declare_parameter("urdf_path", "").value
        self.cam_intrinsics_path = self.declare_parameter(
            "cam_intrinsics_path", "").value
        default_color_tool = (
            "/home/iliana/prog/photorealistic-mapping/src/rove_slam_ros/"
            "scripts/color_mesh.py"
        )
        self.color_mesh_tool = Path(
            self.declare_parameter("color_mesh_tool", default_color_tool).value
        )

        if self.method not in {"poisson", "bpa", "tsdf", "nvblox"}:
            self.get_logger().error(
                f"unknown method '{self.method}', valid: poisson|bpa|tsdf|nvblox"
            )

        self.output_dir.mkdir(parents=True, exist_ok=True)

        self._poses_lock = threading.Lock()
        self._poses: list[tuple[int, list[float], list[float]]] = []
        self._scans: list[tuple[int, np.ndarray]] = []
        self._scans_lock = threading.Lock()
        # Cap on in-memory scan storage so we don't OOM on long runs. Each
        # scan is ~3 MB; 600 scans ≈ 1.8 GB. Drop oldest when over.
        self._max_scans = int(
            self.declare_parameter("max_in_memory_scans", 600).value
        )

        self.create_subscription(
            Odometry, self.odom_topic, self._on_odom, 50
        )
        if not self.recording_dir:
            # Only stash scans when we don't have a .rec to point at.
            self.create_subscription(
                PointCloud2, self.lidar_topic, self._on_lidar,
                qos_profile_sensor_data,
            )

        self.create_service(Trigger, "~/build_mesh", self._handle_build)

        self.get_logger().info(
            f"mesh_builder up: method={self.method} output_dir={self.output_dir} "
            f"recording_dir={self.recording_dir or '<live>'} voxel={self.voxel}"
        )

    # ── data accumulation ─────────────────────────────────────────────

    def _on_odom(self, msg: Odometry) -> None:
        ts = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        with self._poses_lock:
            self._poses.append((ts, [p.x, p.y, p.z],
                                [q.x, q.y, q.z, q.w]))

    def _on_lidar(self, msg: PointCloud2) -> None:
        ts = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        xyzi = _pc2_to_xyzi(msg)
        if xyzi.size == 0:
            return
        with self._scans_lock:
            self._scans.append((ts, xyzi))
            if len(self._scans) > self._max_scans:
                drop = len(self._scans) - self._max_scans
                self._scans = self._scans[drop:]

    # ── service handler ───────────────────────────────────────────────

    def _handle_build(self, _request, response):
        try:
            mesh_path = self._do_build()
            response.success = True
            response.message = str(mesh_path)
        except Exception as e:
            self.get_logger().error(f"build failed: {e}")
            response.success = False
            response.message = str(e)
        return response

    def _do_build(self) -> Path:
        # nvblox runs as a sibling node that has been integrating live;
        # mesh "build" is just a save-to-PLY service call.
        if self.method == "nvblox":
            return self._do_build_nvblox()

        # Snapshot what we have.
        with self._poses_lock:
            poses_snapshot = list(self._poses)
        with self._scans_lock:
            scans_snapshot = list(self._scans)

        if not poses_snapshot:
            raise RuntimeError("no /odom messages received yet")

        traj_path = self.output_dir / "trajectory.tum"
        self._write_tum(traj_path, poses_snapshot)
        self.get_logger().info(
            f"saved {len(poses_snapshot)} poses to {traj_path}"
        )

        # Recording: prefer external; otherwise dump our in-memory scans.
        if self.recording_dir:
            rec_dir = Path(self.recording_dir)
        else:
            if not scans_snapshot:
                raise RuntimeError(
                    "no lidar scans buffered and no recording_dir set"
                )
            rec_dir = self.output_dir / "scans.rec"
            _write_lidar_recording(rec_dir, scans_snapshot)
            self.get_logger().info(
                f"wrote {len(scans_snapshot)} scans to {rec_dir}"
            )

        # Dispatch to build_mesh.py.
        mesh_path = self.output_dir / f"mesh_{self.method}.ply"
        cmd = [sys.executable, str(self.build_mesh_tool),
                "--method", self.method,
                "--rec", str(rec_dir),
                "--traj", str(traj_path),
                "--out", str(mesh_path),
                "--voxel", str(self.voxel)]
        if self.urdf_extrinsic:
            cmd.append("--urdf-extrinsic")
        if self.method == "tsdf":
            cmd += ["--trunc", str(self.tsdf_trunc)]
        elif self.method == "poisson":
            cmd += ["--extra",
                    f"--depth {self.poisson_depth} "
                    f"--density-quantile {self.poisson_dq}"]

        log_path = self.output_dir / "build.log"
        self.get_logger().info(f"running: {' '.join(cmd)}")
        t0 = time.time()
        with open(log_path, "wb") as logf:
            rc = subprocess.call(cmd, stdout=logf, stderr=subprocess.STDOUT)
        elapsed = time.time() - t0
        if rc != 0 or not mesh_path.exists():
            raise RuntimeError(
                f"build_mesh exited {rc} (see {log_path})"
            )
        self.get_logger().info(
            f"built {self.method} mesh in {elapsed:.1f}s → {mesh_path}"
        )

        # Optional: colorize the mesh from a bag's camera streams.
        if self.colorize:
            colored = self._colorize_mesh(mesh_path, traj_path)
            if colored is not None:
                return colored
        return mesh_path

    def _do_build_nvblox(self) -> Path:
        """Save nvblox's live-accumulated mesh via the save_ply service.

        Assumes a `nvblox_node` is already running in the same ROS graph
        (brought up by nvblox.launch.py). nvblox has been integrating
        every scan + image as they arrived — this is purely a save call.
        """
        mesh_path = self.output_dir / "mesh_nvblox.ply"
        try:
            # nvblox_msgs/srv/FilePath: {string file_path → bool success, string message}
            from nvblox_msgs.srv import FilePath
        except ImportError as e:
            raise RuntimeError(
                "nvblox_msgs not available — install isaac_ros_nvblox or "
                "pick a different mesh_method (poisson | tsdf). "
                f"({e})"
            )
        client = self.create_client(FilePath, "/nvblox_node/save_ply")
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(
                "/nvblox_node/save_ply not advertised — is nvblox_node running? "
                "Launch with nvblox.launch.py alongside SLAM."
            )
        req = FilePath.Request()
        req.file_path = str(mesh_path)
        self.get_logger().info(f"requesting nvblox save_ply → {mesh_path}")
        t0 = time.time()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        elapsed = time.time() - t0
        if not future.done() or future.result() is None:
            raise RuntimeError("nvblox save_ply did not return in 30s")
        resp = future.result()
        if not resp.success:
            raise RuntimeError(f"nvblox save_ply failed: {resp.message}")
        self.get_logger().info(
            f"nvblox mesh saved in {elapsed:.1f}s → {mesh_path}"
        )
        # nvblox produces a per-vertex-colored mesh natively, so we
        # skip the color_mesh.py post-process even when colorize=true.
        if self.colorize:
            self.get_logger().info(
                "nvblox mesh is already colored; skipping color_mesh.py."
            )
        return mesh_path

    def _colorize_mesh(self, mesh_path: Path, traj_path: Path) -> Path | None:
        """Run color_mesh.py to project bag camera images onto the mesh."""
        if not self.bag_path:
            self.get_logger().warn(
                "colorize=true but bag_path is empty — skipping colorization."
            )
            return None
        if not self.urdf_path:
            self.get_logger().warn(
                "colorize=true but urdf_path is empty — skipping."
            )
            return None
        if not self.color_mesh_tool.exists():
            self.get_logger().error(
                f"color_mesh tool not found at {self.color_mesh_tool}"
            )
            return None
        out = mesh_path.with_name(mesh_path.stem + "_colored.ply")
        cmd = [sys.executable, str(self.color_mesh_tool),
                "--mesh", str(mesh_path),
                "--bag",  str(self.bag_path),
                "--traj", str(traj_path),
                "--urdf", str(self.urdf_path),
                "--out",  str(out)]
        if self.cam_intrinsics_path:
            cmd += ["--intrinsics", str(self.cam_intrinsics_path)]
        log_path = self.output_dir / "color_mesh.log"
        self.get_logger().info(f"colorizing: {' '.join(cmd)}")
        t0 = time.time()
        with open(log_path, "wb") as logf:
            rc = subprocess.call(cmd, stdout=logf, stderr=subprocess.STDOUT)
        elapsed = time.time() - t0
        if rc != 0 or not out.exists():
            self.get_logger().error(
                f"color_mesh exited {rc} (see {log_path})")
            return None
        self.get_logger().info(
            f"colorized mesh in {elapsed:.1f}s → {out}"
        )
        return out

    def _write_tum(self, path: Path,
                    poses: list[tuple[int, list[float], list[float]]]) -> None:
        # Build the full array first, then atomically write — avoids any
        # partial-line states an interrupted f-string write could leave
        # behind. TUM format: ts tx ty tz qx qy qz qw.
        arr = np.empty((len(poses), 8), dtype=np.float64)
        for i, (ts_ns, xyz, qxyzw) in enumerate(poses):
            arr[i, 0] = float(ts_ns) * 1e-9
            arr[i, 1:4] = xyz
            arr[i, 4:8] = qxyzw
        tmp = path.with_suffix(path.suffix + ".tmp")
        np.savetxt(tmp, arr, fmt="%.9f")
        os.replace(tmp, path)


def main() -> int:
    rclpy.init()
    node = MeshBuilder()
    triggered = {"value": False}

    def _shutdown_build(*_):
        if triggered["value"]:
            return
        triggered["value"] = True
        if node.build_on_shutdown:
            try:
                node.get_logger().info("shutdown: building final mesh…")
                node._do_build()
            except Exception as e:
                node.get_logger().error(f"shutdown build failed: {e}")
        rclpy.shutdown()

    # rclpy's own SIGINT handler runs before ours; install via signal so
    # we get a chance to do the build before the spin loop exits.
    signal.signal(signal.SIGTERM, _shutdown_build)
    signal.signal(signal.SIGINT, _shutdown_build)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        _shutdown_build()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
