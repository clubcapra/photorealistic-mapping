#!/usr/bin/env python3
"""
vectornav_udp_node.py
---------------------
ROS2 node for the Jetson that receives VectorNav VN-300 data from
the Capra Rove sensor interface (running on the Pi) via UDP and
publishes it to standard ROS2 topics.

UDP protocol (from the API spec):
  Packet format: | version (1B) | msg_type (1B) | seq_num (2B LE) | JSON payload |

  msg_type values:
    0x01  Subscribe   (we send to PI to start the stream)
    0x02  Unsubscribe (we send to PI to stop the stream)
    0x03  Data        (PI sends to us continuously)

VectorNav data UDP port: 5000  (PI address configured via ROS param)

Published topics:
  /imu/data_raw      sensor_msgs/Imu          (accel + gyro, no orientation)
  /imu/mag           sensor_msgs/MagneticField (magnetometer)
  /imu/data          sensor_msgs/Imu          (full, with orientation from roll/pitch/yaw)
  /fix               sensor_msgs/NavSatFix    (GPS lat/lon/alt)
  /imu/temperature   sensor_msgs/Temperature
  /imu/pressure      sensor_msgs/FluidPressure
  /vectornav/status  std_msgs/String          (JSON blob of all fields)

Usage:
  ros2 run <your_package> vectornav_udp_node \
      --ros-args -p pi_ip:=192.168.1.100 -p udp_port:=5000

Dependencies (install on Jetson):
  pip3 install transforms3d   # for euler→quaternion
"""

import json
import math
import socket
import struct
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from builtin_interfaces.msg import Time as RosTime
from sensor_msgs.msg import (
    Imu,
    MagneticField,
    NavSatFix,
    NavSatStatus,
    Temperature,
    FluidPressure,
)
from std_msgs.msg import String, Header

# Pure Python euler→quaternion — no external dependencies needed
HAS_TRANSFORMS3D = False  # unused, kept for safety


# ── UDP packet constants ──────────────────────────────────────────────────────
MSG_SUBSCRIBE   = 0x01
MSG_UNSUBSCRIBE = 0x02
MSG_DATA        = 0x03
HEADER_SIZE     = 4   # version(1) + msg_type(1) + seq_num(2)


def build_packet(msg_type: int) -> bytes:
    """Build a minimal Subscribe / Unsubscribe packet (no payload needed)."""
    version  = 1
    seq_num  = 0
    return struct.pack('<BBH', version, msg_type, seq_num)


def parse_packet(raw: bytes):
    """
    Returns (version, msg_type, seq_num, payload_dict | None).
    Returns None on parse errors.
    """
    if len(raw) < HEADER_SIZE:
        return None
    version, msg_type, seq_num = struct.unpack_from('<BBH', raw, 0)
    payload_bytes = raw[HEADER_SIZE:]
    payload = None
    if payload_bytes:
        try:
            payload = json.loads(payload_bytes.decode('utf-8'))
        except Exception:
            pass
    return version, msg_type, seq_num, payload


# ── Helpers ───────────────────────────────────────────────────────────────────
def ros_now(node: Node) -> RosTime:
    t = node.get_clock().now().to_msg()
    return t


def make_header(node: Node, frame_id: str) -> Header:
    h = Header()
    h.stamp = ros_now(node)
    h.frame_id = frame_id
    return h


def euler_to_quaternion(roll_deg, pitch_deg, yaw_deg):
    """
    Convert roll/pitch/yaw (degrees, ZYX/intrinsic) to quaternion (x, y, z, w).
    Pure Python — no external dependencies.
    """
    r = math.radians(roll_deg)  / 2.0
    p = math.radians(pitch_deg) / 2.0
    y = math.radians(yaw_deg)   / 2.0

    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y_ = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (x, y_, z, w)


# ── ROS2 Node ─────────────────────────────────────────────────────────────────
class VectornavUdpNode(Node):

    def __init__(self):
        super().__init__('vectornav_udp_node')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('pi_ip',    '192.168.1.100')
        self.declare_parameter('udp_port', 5000)
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('gps_frame', 'gps_link')
        # How many seconds between re-sending Subscribe (keep-alive)
        self.declare_parameter('subscribe_interval', 5.0)

        self.pi_ip    = self.get_parameter('pi_ip').value
        self.udp_port = self.get_parameter('udp_port').value
        self.imu_frame = self.get_parameter('imu_frame').value
        self.gps_frame = self.get_parameter('gps_frame').value
        sub_interval  = self.get_parameter('subscribe_interval').value

        self.get_logger().info(
            f'Connecting to Pi at {self.pi_ip}:{self.udp_port}')

        # ── QoS ─────────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Publishers ───────────────────────────────────────────────────────
        self.pub_imu_raw  = self.create_publisher(Imu,           '/imu/data_raw',    sensor_qos)
        self.pub_imu      = self.create_publisher(Imu,           '/imu/data',        sensor_qos)
        self.pub_mag      = self.create_publisher(MagneticField, '/imu/mag',         sensor_qos)
        self.pub_fix      = self.create_publisher(NavSatFix,     '/fix',             sensor_qos)
        self.pub_temp     = self.create_publisher(Temperature,   '/imu/temperature', sensor_qos)
        self.pub_pressure = self.create_publisher(FluidPressure, '/imu/pressure',    sensor_qos)
        self.pub_status   = self.create_publisher(String,        '/vectornav/status', 10)

        # ── UDP socket ───────────────────────────────────────────────────────
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', self.udp_port))   # listen on all interfaces
        self.sock.settimeout(1.0)

        # Send initial Subscribe
        self._send_subscribe()

        # Keep-alive timer (re-sends Subscribe periodically)
        self.create_timer(sub_interval, self._send_subscribe)

        # Receive loop in background thread
        self._running = True
        self._rx_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._rx_thread.start()

        self.get_logger().info('VectorNav UDP node started.')

    # ── Subscribe / Unsubscribe ───────────────────────────────────────────────
    def _send_subscribe(self):
        pkt = build_packet(MSG_SUBSCRIBE)
        self.sock.sendto(pkt, (self.pi_ip, self.udp_port))
        self.get_logger().debug('Sent Subscribe to Pi')

    def _send_unsubscribe(self):
        pkt = build_packet(MSG_UNSUBSCRIBE)
        self.sock.sendto(pkt, (self.pi_ip, self.udp_port))
        self.get_logger().debug('Sent Unsubscribe to Pi')

    # ── Receive loop ─────────────────────────────────────────────────────────
    def _receive_loop(self):
        while self._running:
            try:
                raw, _addr = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break

            parsed = parse_packet(raw)
            if parsed is None:
                continue
            _version, msg_type, _seq, payload = parsed

            if msg_type == MSG_DATA and payload:
                try:
                    self._publish(payload)
                except Exception as e:
                    self.get_logger().warn(f'Publish error: {e}')

    # ── Publish helpers ───────────────────────────────────────────────────────
    def _publish(self, d: dict):
        stamp = ros_now(self)

        # ── IMU raw (accel + gyro, no orientation) ──────────────────────────
        imu_raw = Imu()
        imu_raw.header.stamp    = stamp
        imu_raw.header.frame_id = self.imu_frame
        imu_raw.linear_acceleration.x = float(d.get('accel_x', 0.0))
        imu_raw.linear_acceleration.y = float(d.get('accel_y', 0.0))
        imu_raw.linear_acceleration.z = float(d.get('accel_z', 0.0))
        imu_raw.angular_velocity.x    = float(d.get('gyro_x',  0.0))
        imu_raw.angular_velocity.y    = float(d.get('gyro_y',  0.0))
        imu_raw.angular_velocity.z    = float(d.get('gyro_z',  0.0))
        # Mark covariances unknown (-1 in first element = unknown per REP-145)
        UNKNOWN_COV = np.array([-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=float)
        imu_raw.orientation_covariance[:]         = UNKNOWN_COV
        imu_raw.linear_acceleration_covariance[:] = UNKNOWN_COV
        imu_raw.angular_velocity_covariance[:]    = UNKNOWN_COV
        self.pub_imu_raw.publish(imu_raw)

        # ── IMU full (with orientation from roll/pitch/yaw) ─────────────────
        imu = Imu()
        imu.header.stamp    = stamp
        imu.header.frame_id = self.imu_frame
        imu.linear_acceleration.x = imu_raw.linear_acceleration.x
        imu.linear_acceleration.y = imu_raw.linear_acceleration.y
        imu.linear_acceleration.z = imu_raw.linear_acceleration.z
        imu.angular_velocity.x    = imu_raw.angular_velocity.x
        imu.angular_velocity.y    = imu_raw.angular_velocity.y
        imu.angular_velocity.z    = imu_raw.angular_velocity.z

        roll  = float(d.get('roll',  0.0))
        pitch = float(d.get('pitch', 0.0))
        yaw   = float(d.get('yaw',   0.0))
        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.orientation.w = qw

        att_unc = float(d.get('att_uncertainty', -1.0))
        if att_unc > 0.0:
            var = float(att_unc ** 2)
            imu.orientation_covariance[:] = np.array([
                var,  0.0, 0.0,
                0.0,  var, 0.0,
                0.0,  0.0, var,
            ], dtype=float)
        else:
            imu.orientation_covariance[:] = UNKNOWN_COV

        imu.linear_acceleration_covariance[:] = UNKNOWN_COV
        imu.angular_velocity_covariance[:]    = UNKNOWN_COV
        self.pub_imu.publish(imu)

        # ── Magnetometer ────────────────────────────────────────────────────
        mag = MagneticField()
        mag.header.stamp    = stamp
        mag.header.frame_id = self.imu_frame
        mag.magnetic_field.x = float(d.get('mag_x', 0.0))
        mag.magnetic_field.y = float(d.get('mag_y', 0.0))
        mag.magnetic_field.z = float(d.get('mag_z', 0.0))
        mag.magnetic_field_covariance[:] = UNKNOWN_COV
        self.pub_mag.publish(mag)

        # ── GPS fix ─────────────────────────────────────────────────────────
        fix = NavSatFix()
        fix.header.stamp    = stamp
        fix.header.frame_id = self.gps_frame
        fix.latitude  = float(d.get('latitude',  0.0))
        fix.longitude = float(d.get('longitude', 0.0))
        fix.altitude  = float(d.get('altitude',  0.0))

        gnss_fix = d.get('gnss_fix', False)
        fix_type = int(d.get('gnss_fix_type', 0))
        if not gnss_fix:
            fix.status.status = NavSatStatus.STATUS_NO_FIX
        elif fix_type >= 4:                           # RTK fixed
            fix.status.status = NavSatStatus.STATUS_GBAS_FIX
        elif fix_type >= 3:                           # RTK float / SBAS
            fix.status.status = NavSatStatus.STATUS_SBAS_FIX
        else:
            fix.status.status = NavSatStatus.STATUS_FIX

        fix.status.service = NavSatStatus.SERVICE_GPS
        pos_unc = float(d.get('pos_uncertainty', -1.0))
        if pos_unc >= 0.0:
            var = float(pos_unc ** 2)
            fix.position_covariance[:] = np.array([
                var,  0.0, 0.0,
                0.0,  var, 0.0,
                0.0,  0.0, var * 4.0,
            ], dtype=float)
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        else:
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.pub_fix.publish(fix)

        # ── Temperature ─────────────────────────────────────────────────────
        if 'temperature' in d:
            temp = Temperature()
            temp.header.stamp    = stamp
            temp.header.frame_id = self.imu_frame
            temp.temperature     = float(d['temperature'])
            temp.variance        = 0.0
            self.pub_temp.publish(temp)

        # ── Pressure ────────────────────────────────────────────────────────
        if 'pressure' in d:
            pres = FluidPressure()
            pres.header.stamp    = stamp
            pres.header.frame_id = self.imu_frame
            pres.fluid_pressure  = float(d['pressure'])  # kPa from VN-300
            pres.variance        = 0.0
            self.pub_pressure.publish(pres)

        # ── Raw status JSON ──────────────────────────────────────────────────
        status_msg = String()
        status_msg.data = json.dumps({
            'ins_mode':          d.get('ins_mode'),
            'ins_error':         d.get('ins_error'),
            'ins_status_raw':    d.get('ins_status_raw'),
            'gnss_fix':          d.get('gnss_fix'),
            'gnss_fix_type':     d.get('gnss_fix_type'),
            'gnss_num_sats':     d.get('gnss_num_sats'),
            'gnss_compass_active':  d.get('gnss_compass_active'),
            'gnss_heading_aiding':  d.get('gnss_heading_aiding'),
            'gps_week':          d.get('gps_week'),
            'gps_tow':           d.get('gps_tow'),
            'messages_parsed':   d.get('messages_parsed'),
            'messages_dropped':  d.get('messages_dropped'),
            'last_async_header': d.get('last_async_header'),
        })
        self.pub_status.publish(status_msg)

    # ── Cleanup ───────────────────────────────────────────────────────────────
    def destroy_node(self):
        self._running = False
        self._send_unsubscribe()
        self.sock.close()
        super().destroy_node()


# ── Entry point ───────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = VectornavUdpNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()