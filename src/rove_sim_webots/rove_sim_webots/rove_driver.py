"""Webots ROS 2 driver plugin for the Rove rover.

Loaded by webots_ros2_driver via the <plugin type="..."> tag in
urdf/rove_webots.urdf.

Responsibilities:
- /cmd_vel -> 4-wheel skid-steer motor commands.
- Publish nav_msgs/Odometry on /odom (wheel-cmd integrated pose).
- Broadcast tf odom -> base_link continuously.
- Broadcast static tf base_link -> livox_frame and base_link -> camera_link once.
- (Supervisor mode) Publish nav_msgs/Odometry on /ground_truth/odom with the
  robot's TRUE pose queried from Webots — used by validator.py to cross-check
  RTAB-Map's estimated trajectory against the simulator's ground truth.

IMU publication is NOT handled here — the standard Ros2IMU plugin in the URDF
composes a single sensor_msgs/Imu on /livox/imu from the InertialUnit +
Accelerometer + Gyro devices.
Lidar publication is handled by webots_ros2_driver's built-in Lidar device
wrapper (configured by the URDF <device reference="rove_lidar"> block).
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def _rotmat_to_quat(rot9) -> Quaternion:
    """Webots row-major 3x3 rotation matrix (9 floats) -> Quaternion."""
    r11, r12, r13, r21, r22, r23, r31, r32, r33 = rot9
    trace = r11 + r22 + r33
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (r32 - r23) * s
        qy = (r13 - r31) * s
        qz = (r21 - r12) * s
    elif r11 > r22 and r11 > r33:
        s = 2.0 * math.sqrt(1.0 + r11 - r22 - r33)
        qw = (r32 - r23) / s
        qx = 0.25 * s
        qy = (r12 + r21) / s
        qz = (r13 + r31) / s
    elif r22 > r33:
        s = 2.0 * math.sqrt(1.0 + r22 - r11 - r33)
        qw = (r13 - r31) / s
        qx = (r12 + r21) / s
        qy = 0.25 * s
        qz = (r23 + r32) / s
    else:
        s = 2.0 * math.sqrt(1.0 + r33 - r11 - r22)
        qw = (r21 - r12) / s
        qx = (r13 + r31) / s
        qy = (r23 + r32) / s
        qz = 0.25 * s
    q = Quaternion()
    q.x, q.y, q.z, q.w = qx, qy, qz, qw
    return q


def _make_static_tf(parent: str, child: str, xyz, rpy) -> TransformStamped:
    t = TransformStamped()
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
    roll, pitch, yaw = rpy
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    t.transform.rotation.w = cr * cp * cy + sr * sp * sy
    t.transform.rotation.x = sr * cp * cy - cr * sp * sy
    t.transform.rotation.y = cr * sp * cy + sr * cp * sy
    t.transform.rotation.z = cr * cp * sy - sr * sp * cy
    return t


class RoveDriver:
    """webots_ros2_driver plugin class."""

    def init(self, webots_node, properties):
        self._robot = webots_node.robot

        timestep = int(self._robot.getBasicTimeStep())

        # Wheels (4-wheel skid-steer).
        self._motors = {
            name: self._robot.getDevice(name)
            for name in (
                'left_front_motor', 'left_rear_motor',
                'right_front_motor', 'right_rear_motor',
            )
        }
        for m in self._motors.values():
            m.setPosition(float('inf'))
            m.setVelocity(0.0)

        # ROS 2 node.
        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = rclpy.create_node('rove_sim_driver')
        # Inherit sim time — without this, message stamps are wall-clock while
        # rtabmap's are sim-time, and validator can't pair gt with estimate.
        from rclpy.parameter import Parameter as _Param
        self._node.set_parameters([_Param('use_sim_time', _Param.Type.BOOL, True)])
        self._logger = self._node.get_logger()

        self._wheel_base = float(properties.get('wheel_base', 0.63))
        self._wheel_radius = float(properties.get('wheel_radius', 0.12))
        self._base_frame = properties.get('base_frame', 'base_link')
        self._odom_frame = properties.get('odom_frame', 'odom')
        self._lidar_frame = properties.get('lidar_frame', 'livox_frame')
        publish_static = str(properties.get('publish_static_tf', 'true')).lower() == 'true'
        # When SLAM runs alongside the sim, SLAM owns odom -> base_link. If both
        # publishers write the same TF the buffer alternates between them and
        # the robot position jumps. Default OFF - the wheel-integrated estimate
        # still goes out on /odom as a topic for downstream fusers.
        self._publish_odom_tf = (
            str(properties.get('publish_odom_tf', 'false')).lower() == 'true'
        )

        cmd_vel_topic = properties.get('cmd_vel_topic', '/cmd_vel')
        odom_topic = properties.get('odom_topic', '/odom')
        self._node.create_subscription(Twist, cmd_vel_topic, self._on_cmd_vel, 10)
        self._odom_pub = self._node.create_publisher(Odometry, odom_topic, 10)
        self._logger.info(
            f'RoveDriver subscribed to {cmd_vel_topic}, publishing on {odom_topic}'
        )

        # Ground-truth publishing (supervisor required; Rove.proto sets supervisor TRUE).
        # Validator post-processes /ground_truth/odom against RTAB-Map's estimate.
        publish_gt = str(properties.get('publish_ground_truth', 'true')).lower() == 'true'
        self._gt_node = None
        self._gt_pub = None
        self._world_frame = properties.get('world_frame', 'world')
        gt_topic = properties.get('ground_truth_topic', '/ground_truth/odom')
        if publish_gt:
            try:
                self._gt_node = self._robot.getSelf()  # supervisor-only
            except Exception as e:
                self._logger.warn(
                    f'Ground-truth disabled — supervisor handle unavailable: {e}'
                )
            if self._gt_node is not None:
                self._gt_pub = self._node.create_publisher(Odometry, gt_topic, 10)
                self._gt_pose_prev = None  # (x, y, z, yaw) for finite-diff velocity
                self._logger.info(f'Ground-truth publishing enabled ({gt_topic})')

        self._tf = TransformBroadcaster(self._node)
        if publish_static:
            self._static_tf = StaticTransformBroadcaster(self._node)
            # Two lidars matching rove_standard.urdf's dual-MID-360 mount.
            # Pose chain: Core ← pole_pivot ← pole ← dji_mid360*_pivot ← dji_mid360*.
            # Composed in Python (see project notes); the proto's LIDAR_TOP /
            # LIDAR_BOTTOM Pose blocks use the same xyz/rpy.
            # Real-Rove mount: TOP upright, BOTTOM upside-down (180deg roll
            # about X). XYZ matches rove_standard.urdf.
            # Camera TFs are NOT published here — robot_state_publisher
            # consumes rove_standard.urdf and publishes the full chain to
            # cam_<dir>_optical_frame on its own.
            self._static_tf.sendTransform([
                _make_static_tf(
                    self._base_frame, self._lidar_frame,       # top -> livox_frame
                    xyz=(0.2722, 0.2084, 0.8343),
                    rpy=(0.0, 0.0, 0.0),
                ),
                _make_static_tf(
                    self._base_frame, 'livox_frame_2',         # bottom, upside-down
                    xyz=(0.2702, 0.2090, 0.7638),
                    rpy=(math.pi, 0.0, 0.0),
                ),
            ])

        # Integration state.
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._last_time = self._node.get_clock().now()

        # Latest cmd_vel.
        self._target_linear = 0.0
        self._target_angular = 0.0

        self._logger.info('RoveDriver initialized')

    def _on_cmd_vel(self, msg: Twist):
        self._target_linear = msg.linear.x
        self._target_angular = msg.angular.z

    def step(self):
        rclpy.spin_once(self._node, timeout_sec=0.0)

        v = self._target_linear
        w = self._target_angular
        v_left = v - w * self._wheel_base / 2.0
        v_right = v + w * self._wheel_base / 2.0
        omega_left = v_left / self._wheel_radius
        omega_right = v_right / self._wheel_radius

        self._motors['left_front_motor'].setVelocity(omega_left)
        self._motors['left_rear_motor'].setVelocity(omega_left)
        self._motors['right_front_motor'].setVelocity(omega_right)
        self._motors['right_rear_motor'].setVelocity(omega_right)

        now = self._node.get_clock().now()
        dt = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now
        if dt <= 0:
            return

        self._yaw += w * dt
        self._x += v * math.cos(self._yaw) * dt
        self._y += v * math.sin(self._yaw) * dt

        stamp = now.to_msg()
        q = _yaw_to_quat(self._yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self._odom_pub.publish(odom)

        if self._publish_odom_tf:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self._odom_frame
            tf.child_frame_id = self._base_frame
            tf.transform.translation.x = self._x
            tf.transform.translation.y = self._y
            tf.transform.rotation = q
            self._tf.sendTransform(tf)

        # Ground truth via supervisor.
        if self._gt_pub is not None and self._gt_node is not None:
            pos = self._gt_node.getPosition()
            rot = self._gt_node.getOrientation()
            gt = Odometry()
            gt.header.stamp = stamp
            gt.header.frame_id = self._world_frame
            gt.child_frame_id = self._base_frame
            gt.pose.pose.position.x = float(pos[0])
            gt.pose.pose.position.y = float(pos[1])
            gt.pose.pose.position.z = float(pos[2])
            gt.pose.pose.orientation = _rotmat_to_quat(rot)
            # Finite-diff velocity for completeness.
            if self._gt_pose_prev is not None and dt > 0:
                px, py, pz, _pyaw = self._gt_pose_prev
                gt.twist.twist.linear.x = (pos[0] - px) / dt
                gt.twist.twist.linear.y = (pos[1] - py) / dt
                gt.twist.twist.linear.z = (pos[2] - pz) / dt
            self._gt_pose_prev = (pos[0], pos[1], pos[2], 0.0)
            self._gt_pub.publish(gt)
