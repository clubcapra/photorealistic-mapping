#!/usr/bin/env python3
"""Fisheye (equidistant / Kannala-Brandt) rectification node.

RTAB-Map's RGB path assumes already-rectified images, and gscam2 only
*publishes* CameraInfo — it never undistorts. The cardinal cameras are
fisheye (equidistant), so feeding their raw frames to rtabmap leaves the
distortion in place. This node sits between gscam2 and rtabmap:

    <ns>/image_raw  + <ns>/camera_info   (equidistant)
        -> cv2.fisheye.undistort (remap)
    <ns>/image_rect + <ns>/camera_info_rect   (pinhole, zero distortion)

The output is a real topic, so the rectified frame can be inspected
directly in rviz (Image display) and the lidar can be projected onto it
to eyeball the camera/lidar overlap.

K and D are read from the incoming CameraInfo topic (not a static file),
so the node always rectifies with exactly what gscam publishes; the
undistort maps are (re)built whenever the CameraInfo changes.

Parameters:
    cameras    (string[])  namespaces to rectify, e.g. ['cam_north', ...]
    balance    (double)    cv2.fisheye balance, 0=crop to valid pixels,
                           1=keep all source pixels (black borders). 0.0 default.
    fov_scale  (double)    fov scale for the new camera matrix (1.0 default).
    image_qos  (string)    'sensor_data' (best-effort, default) or 'reliable'.
"""

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class _CamRectifier:
    """Per-camera state: caches CameraInfo, builds maps, rectifies frames."""

    def __init__(self, node: 'FisheyeRectify', ns: str, image_qos):
        self.node = node
        self.ns = ns
        self.bridge = node.bridge

        self.map1 = None
        self.map2 = None
        self.rect_info: CameraInfo | None = None
        self._last_K = None
        self._last_D = None

        self.pub_img = node.create_publisher(
            Image, f'{ns}/image_rect', image_qos)
        self.pub_info = node.create_publisher(
            CameraInfo, f'{ns}/camera_info_rect', 10)

        self.sub_info = node.create_subscription(
            CameraInfo, f'{ns}/camera_info', self.on_info, 10)
        self.sub_img = node.create_subscription(
            Image, f'{ns}/image_raw', self.on_image, image_qos)

        node.get_logger().info(
            f'[{ns}] rectifier up: {ns}/image_raw -> {ns}/image_rect')

    # -- CameraInfo: (re)build undistort maps -------------------------------
    def on_info(self, msg: CameraInfo):
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        D = np.array(msg.d, dtype=np.float64)

        model = (msg.distortion_model or '').lower()
        if model != 'equidistant':
            if self.map1 is None:  # warn once
                self.node.get_logger().warn(
                    f'[{self.ns}] distortion_model="{msg.distortion_model}" '
                    f'is not "equidistant"; passing frames through unrectified.')
            self.rect_info = msg          # forward original info as-is
            self.map1 = self.map2 = None  # signal passthrough
            return

        # Only rebuild when the calibration actually changes.
        if (self._last_K is not None
                and np.array_equal(self._last_K, K)
                and np.array_equal(self._last_D, D)):
            return

        W, H = msg.width, msg.height
        Dk = D[:4].reshape(4, 1)  # fisheye wants exactly 4 coeffs
        new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
            K, Dk, (W, H), np.eye(3),
            balance=self.node.balance, fov_scale=self.node.fov_scale)
        self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
            K, Dk, np.eye(3), new_K, (W, H), cv2.CV_16SC2)
        self._last_K, self._last_D = K.copy(), D.copy()

        # Rectified CameraInfo: pinhole, zero distortion, identity R.
        info = CameraInfo()
        info.header = msg.header
        info.width, info.height = W, H
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = new_K.flatten().tolist()
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        P = np.zeros((3, 4))
        P[:3, :3] = new_K
        info.p = P.flatten().tolist()
        self.rect_info = info
        self.node.get_logger().info(
            f'[{self.ns}] undistort maps built ({W}x{H}, '
            f'balance={self.node.balance}).')

    # -- Image: rectify + republish -----------------------------------------
    def on_image(self, msg: Image):
        if self.rect_info is None:
            return  # no CameraInfo yet
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        if self.map1 is None:  # passthrough (non-fisheye / not ready)
            out = msg
        else:
            rect = cv2.remap(img, self.map1, self.map2,
                             interpolation=cv2.INTER_LINEAR,
                             borderMode=cv2.BORDER_CONSTANT)
            out = self.bridge.cv2_to_imgmsg(rect, encoding=msg.encoding)
            out.header = msg.header  # preserve stamp + frame_id

        self.pub_img.publish(out)
        # Stamp the rectified CameraInfo to match this frame exactly.
        self.rect_info.header = msg.header
        self.pub_info.publish(self.rect_info)


class FisheyeRectify(Node):
    def __init__(self):
        super().__init__('fisheye_rectify')
        self.bridge = CvBridge()

        self.declare_parameter('cameras', ['cam_north'])
        self.declare_parameter('balance', 0.0)
        self.declare_parameter('fov_scale', 1.0)
        self.declare_parameter('image_qos', 'sensor_data')

        cams = self.get_parameter('cameras').value
        self.balance = float(self.get_parameter('balance').value)
        self.fov_scale = float(self.get_parameter('fov_scale').value)

        qos_name = self.get_parameter('image_qos').value
        if qos_name == 'reliable':
            image_qos = QoSProfile(depth=10,
                                   reliability=ReliabilityPolicy.RELIABLE)
        else:
            image_qos = qos_profile_sensor_data

        self.rectifiers = [_CamRectifier(self, ns, image_qos) for ns in cams]
        self.get_logger().info(f'fisheye_rectify rectifying: {list(cams)}')


def main(args=None):
    rclpy.init(args=args)
    node = FisheyeRectify()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
