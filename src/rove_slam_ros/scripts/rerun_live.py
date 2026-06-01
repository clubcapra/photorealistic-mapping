#!/usr/bin/env python3
"""Live rerun viewer for rove_slam.

A rclpy node that subscribes to the SLAM bridge's outputs and logs them
to rerun in real time. The viewer pops up automatically (or `--serve`
to expose the gRPC port and connect with a separate `rerun` viewer).

Topics consumed (all optional — node skips silently if missing):
  /tf, /tf_static          tf2_msgs/TFMessage    — TF tree, drives a
                                                    moving rover entity
  /odom                    nav_msgs/Odometry     — rover trajectory polyline
  /cloud_obstacles         sensor_msgs/PointCloud2 — Z-banded local obstacles
  /livox/lidar             sensor_msgs/PointCloud2 — raw lidar (optional,
                                                    expensive; --no-raw to skip)

Parameters:
  ~spawn (bool, default true)   pop up the viewer window automatically
  ~serve (bool, default false)  start a gRPC server instead; connect with
                                `rerun --connect rerun+http://127.0.0.1:9876/proxy`
  ~raw (bool, default false)    also stream /livox/lidar (heavy)
  ~max_traj_pts (int, default 5000)  cap trajectory polyline length

Install:
  pip install --user rerun-sdk    # rerun 0.33 is what the project uses

Run alongside SLAM:
  ros2 run rove_slam_ros rerun_live.py
or via bringup.launch.py with `viewer:=true`.
"""
from __future__ import annotations

import struct
import sys
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage

try:
    import rerun as rr
except ImportError:
    sys.stderr.write(
        "rerun is not installed. Install with: pip install --user 'rerun-sdk==0.21.0'\n"
        "(0.21 is the last rerun that supports numpy<2, which ROS Humble Python\n"
        "bindings need.)\n"
    )
    sys.exit(2)


def _set_timeline(name: str, seconds: float) -> None:
    """rerun 0.21 / 0.33 differ in their time API. Try the new one first."""
    if hasattr(rr, "set_time"):
        try:
            rr.set_time(name, duration=seconds)
            return
        except Exception:
            pass
    if hasattr(rr, "set_time_seconds"):
        rr.set_time_seconds(name, seconds)


def _pc2_to_xyz(msg: PointCloud2) -> np.ndarray:
    """Decode a PointCloud2 to an (N, 3) float32 array. Handles XYZ +
    arbitrary trailing fields by stepping `point_step` bytes per point."""
    off_x = off_y = off_z = -1
    for fd in msg.fields:
        if fd.name == "x": off_x = fd.offset
        elif fd.name == "y": off_y = fd.offset
        elif fd.name == "z": off_z = fd.offset
    if off_x < 0 or off_y < 0 or off_z < 0:
        return np.empty((0, 3), dtype=np.float32)

    n = msg.width * msg.height
    if n == 0:
        return np.empty((0, 3), dtype=np.float32)
    raw = bytes(msg.data)
    step = msg.point_step
    if off_x == 0 and off_y == 4 and off_z == 8 and step >= 12:
        # Common dense layout — vectorized decode.
        arr = np.frombuffer(raw, dtype=np.uint8)
        arr = arr.reshape(n, step)[:, :12].copy()
        return np.frombuffer(arr.tobytes(), dtype=np.float32).reshape(n, 3)
    # General fallback.
    out = np.empty((n, 3), dtype=np.float32)
    for i in range(n):
        base = i * step
        out[i, 0] = struct.unpack_from("<f", raw, base + off_x)[0]
        out[i, 1] = struct.unpack_from("<f", raw, base + off_y)[0]
        out[i, 2] = struct.unpack_from("<f", raw, base + off_z)[0]
    return out


def _stamp_secs(stamp) -> float:
    return float(stamp.sec) + stamp.nanosec * 1e-9


class RerunLiveNode(Node):
    def __init__(self) -> None:
        super().__init__("rerun_live")
        spawn = self.declare_parameter("spawn", True).value
        serve = self.declare_parameter("serve", False).value
        self._raw = self.declare_parameter("raw", False).value
        self._max_traj = int(self.declare_parameter("max_traj_pts", 5000).value)

        rr.init("rove_slam_live", spawn=(spawn and not serve))
        if serve:
            # rerun 0.21 uses rr.serve(); 0.30+ renamed to serve_grpc.
            (rr.serve_grpc if hasattr(rr, "serve_grpc") else rr.serve)()
            self.get_logger().info(
                "rerun serving — connect with: rerun --connect 127.0.0.1:9876"
            )
        rr.log("/", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

        # Trajectory buffer.
        self._traj: deque[tuple[float, float, float]] = deque(maxlen=self._max_traj)

        qos = qos_profile_sensor_data
        self.create_subscription(TFMessage, "/tf", self.on_tf, 50)
        self.create_subscription(TFMessage, "/tf_static", self.on_tf, 1)
        self.create_subscription(Odometry, "/odom", self.on_odom, 50)
        self.create_subscription(
            PointCloud2, "/cloud_obstacles", self.on_obstacles, qos
        )
        if self._raw:
            self.create_subscription(
                PointCloud2, "/livox/lidar", self.on_raw, qos
            )

        self.get_logger().info(
            f"rerun_live up: spawn={spawn} serve={serve} raw={self._raw} "
            f"max_traj={self._max_traj}"
        )

    # ── callbacks ────────────────────────────────────────────────────

    def on_tf(self, msg: TFMessage) -> None:
        for tf in msg.transforms:
            t = tf.transform.translation
            q = tf.transform.rotation
            _set_timeline("scan_time", _stamp_secs(tf.header.stamp))
            ent = f"world/tf/{tf.header.frame_id}/{tf.child_frame_id}"
            rr.log(
                ent,
                rr.Transform3D(
                    translation=[t.x, t.y, t.z],
                    rotation=rr.Quaternion(xyzw=[q.x, q.y, q.z, q.w]),
                ),
            )

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _set_timeline("scan_time", _stamp_secs(msg.header.stamp))
        # Rover marker (transform + axes).
        rr.log(
            "world/rover",
            rr.Transform3D(
                translation=[p.x, p.y, p.z],
                rotation=rr.Quaternion(xyzw=[q.x, q.y, q.z, q.w]),
            ),
        )
        rr.log(
            "world/rover/axes",
            rr.Arrows3D(
                origins=[[0, 0, 0]] * 3,
                vectors=[[0.4, 0, 0], [0, 0.4, 0], [0, 0, 0.4]],
                colors=[[255, 50, 50], [50, 255, 50], [50, 50, 255]],
            ),
        )
        # Trajectory polyline (accumulating).
        self._traj.append((p.x, p.y, p.z))
        if len(self._traj) >= 2:
            pts = np.asarray(self._traj, dtype=np.float32)
            rr.log(
                "world/trajectory",
                rr.LineStrips3D(pts, colors=[60, 130, 250], radii=0.03),
            )

    def on_obstacles(self, msg: PointCloud2) -> None:
        xyz = _pc2_to_xyz(msg)
        if xyz.size == 0:
            return
        _set_timeline("scan_time", _stamp_secs(msg.header.stamp))
        # Colour by Z.
        z = xyz[:, 2]
        zmin, zmax = float(z.min()), float(z.max())
        zspan = max(zmax - zmin, 1e-6)
        t = (z - zmin) / zspan
        import matplotlib.cm as cm
        cols = (cm.plasma(t)[:, :3] * 255).astype(np.uint8)
        rr.log(
            "world/cloud_obstacles",
            rr.Points3D(xyz, colors=cols, radii=0.02),
        )

    def on_raw(self, msg: PointCloud2) -> None:
        xyz = _pc2_to_xyz(msg)
        if xyz.size == 0:
            return
        # Heavy — subsample for visualization.
        if len(xyz) > 30000:
            idx = np.random.choice(len(xyz), 30000, replace=False)
            xyz = xyz[idx]
        _set_timeline("scan_time", _stamp_secs(msg.header.stamp))
        rr.log(
            "lidar/raw",
            rr.Points3D(xyz, colors=[180, 180, 200], radii=0.015),
        )


def main() -> int:
    rclpy.init()
    node = RerunLiveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
