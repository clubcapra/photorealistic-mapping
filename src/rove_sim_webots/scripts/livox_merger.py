#!/usr/bin/env python3
"""Merge two MID-360 point clouds into a single /livox/lidar stream.

The real Rove has two lidars mounted with opposite +/-90deg roll so their
hemispheric scans combine into near-spherical coverage. KISS-ICP wants a
single PointCloud2 input; this node:
  - Subscribes to /livox/lidar_192_168_2_40 (TOP, frame=livox_frame)
  - Subscribes to /livox/lidar_192_168_2_41 (BOTTOM, frame=livox_frame_2)
  - Transforms BOTTOM points into livox_frame using the static URDF transform
    composed at startup (no per-frame TF lookups - both mounts are rigid).
  - Concatenates and publishes on /livox/lidar (frame=livox_frame).

Pairing uses message time: emit when either side updates, with the most
recent BOTTOM frame transformed and stitched on. The two lidars run at the
same rate so latency stays bounded.
"""
from __future__ import annotations

import struct

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField


# Static transform from livox_frame_2 -> livox_frame. Real-Rove mount: TOP
# lidar is upright (livox_frame), BOTTOM is upside-down (livox_frame_2 has
# Rx(180deg) relative to base_link).
#   base_link -> livox_frame:    xyz=(0.2722, 0.2084, 0.8343), rpy=(0,0,0)
#   base_link -> livox_frame_2:  xyz=(0.2702, 0.2090, 0.7638), rpy=(pi,0,0)
# A point P_lf2 maps to P_lf via P_lf = Rx(pi) @ P_lf2 + (p_lf2 - p_lf).
_ROT_LF2_TO_LF = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], dtype=np.float32)
_TRANS_LF2_TO_LF = np.array([-0.0020, 0.0006, -0.0705], dtype=np.float32)


def _read_xyz(msg: PointCloud2) -> np.ndarray:
    """Extract Nx3 float32 XYZ, dropping NaN/inf points. Webots emits NaN for
    rays that miss everything within maxRange."""
    pt_step = msg.point_step
    n = msg.width * msg.height
    if n == 0:
        return np.empty((0, 3), dtype=np.float32)
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    arr = buf[: n * pt_step].reshape(n, pt_step)
    xyz_bytes = arr[:, :12].tobytes()
    pts = np.frombuffer(xyz_bytes, dtype=np.float32).reshape(n, 3)
    return pts[np.isfinite(pts).all(axis=1)]


def _voxel_density_filter(pts: np.ndarray,
                          voxel_size: float = 0.25,
                          min_pts_per_voxel: int = 3) -> np.ndarray:
    """Drop points whose voxel bin contains fewer than min_pts_per_voxel.
    Removes stray one-off returns that produce ghost obstacles in the costmap.
    Original point coordinates are preserved (no downsampling) so wall geometry
    stays sharp. O(N) via numpy.unique inverse indices.

    Density vs distance for the sim's 900x16 lidar at voxel=0.25 m:
      d=3 m: ~14 pts/voxel    d=5 m: ~10 pts/voxel
      d=8 m: ~4 pts/voxel     d=10 m: ~2.5 pts/voxel
    Min=3 drops singletons but keeps walls out to ~8 m, matching the nav2
    costmap obstacle_max_range of 8 m on the local costmap."""
    if pts.shape[0] == 0:
        return pts
    keys = np.floor(pts / voxel_size).astype(np.int64)
    h = keys[:, 0] * 1_000_003 + keys[:, 1] * 1_009 + keys[:, 2]
    _, inv, counts = np.unique(h, return_inverse=True, return_counts=True)
    return pts[counts[inv] >= min_pts_per_voxel]


def _make_pc2(stamp, frame_id: str, pts: np.ndarray) -> PointCloud2:
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = pts.shape[0]
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * pts.shape[0]
    msg.is_dense = True
    msg.data = pts.astype(np.float32).tobytes()
    return msg


class LivoxMerger(Node):
    def __init__(self):
        super().__init__('livox_merger')
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._bottom_pts: np.ndarray | None = None
        self._pub = self.create_publisher(PointCloud2, '/livox/lidar', qos)
        self.create_subscription(
            PointCloud2, '/livox/lidar_192_168_2_40', self._on_top, qos)
        self.create_subscription(
            PointCloud2, '/livox/lidar_192_168_2_41', self._on_bottom, qos)
        self.get_logger().info(
            'livox_merger: combining /livox/lidar_192_168_2_{40,41} -> /livox/lidar')

    def _on_bottom(self, msg: PointCloud2):
        pts = _read_xyz(msg)
        if pts.size:
            self._bottom_pts = (_ROT_LF2_TO_LF @ pts.T).T + _TRANS_LF2_TO_LF
        else:
            self._bottom_pts = pts

    def _on_top(self, msg: PointCloud2):
        top_pts = _read_xyz(msg)
        parts = [top_pts]
        if self._bottom_pts is not None and self._bottom_pts.size:
            parts.append(self._bottom_pts)
        combined = np.concatenate(parts, axis=0) if parts else top_pts
        combined = _voxel_density_filter(combined)
        self._pub.publish(_make_pc2(msg.header.stamp, 'livox_frame', combined))


def main():
    rclpy.init()
    node = LivoxMerger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
