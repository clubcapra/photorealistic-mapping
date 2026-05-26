"""Closed-loop waypoint driver.

Subscribes to /ground_truth/odom for feedback, publishes /cmd_vel to drive
the robot through a list of (x, y) waypoints. Replaces the open-loop
sequence in scripted_runner for long trajectories where skid-steer drift
accumulates.

YAML format (waypoints variant):
    name: my_traj
    type: waypoints
    waypoints:
      - [1.0, 2.0]
      - [3.5, -1.2]
    yaw_tol_rad: 0.10        # optional, default 0.10
    pos_tol_m: 0.30          # optional, default 0.30
    max_v: 0.4
    max_w: 0.5
"""
from __future__ import annotations

import math
import sys
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter


def quat_to_yaw(qx, qy, qz, qw):
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


class WaypointDriver(Node):
    def __init__(self, traj: dict, use_sim_time: bool = True):
        super().__init__(
            'waypoint_driver',
            parameter_overrides=[Parameter('use_sim_time', value=use_sim_time)],
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self._odom_cb, 10)

        self.waypoints: list[tuple[float, float]] = [tuple(p) for p in traj['waypoints']]
        self.pos_tol = float(traj.get('pos_tol_m', 0.30))
        self.yaw_tol = float(traj.get('yaw_tol_rad', 0.10))
        self.max_v = float(traj.get('max_v', 0.4))
        self.max_w = float(traj.get('max_w', 0.5))
        # Hard stop if a waypoint isn't reached within this many seconds of
        # progress (no closer-than-current distance for this many sec).
        self.stuck_timeout_s = float(traj.get('stuck_timeout_s', 8.0))

        self.x = self.y = self.yaw = None
        self.have_odom = False
        self.cur_wp_idx = 0
        self.best_dist_to_wp = float('inf')
        self.last_progress_t = self.get_clock().now()

        self.done = False
        self.create_timer(0.05, self._control_step)
        self.get_logger().info(
            f'waypoint_driver: {len(self.waypoints)} waypoints, '
            f'pos_tol={self.pos_tol}m yaw_tol={self.yaw_tol}rad '
            f'max_v={self.max_v} max_w={self.max_w}'
        )

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x, self.y = p.x, p.y
        self.yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self.have_odom = True

    def _publish(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def _stop(self):
        self._publish(0.0, 0.0)

    def _control_step(self):
        if self.done or not self.have_odom:
            return
        if self.cur_wp_idx >= len(self.waypoints):
            self._stop()
            self.done = True
            self.get_logger().info('all waypoints reached')
            return

        wx, wy = self.waypoints[self.cur_wp_idx]
        dx, dy = wx - self.x, wy - self.y
        dist = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)
        yaw_err = (target_yaw - self.yaw + math.pi) % (2 * math.pi) - math.pi

        if dist < self.pos_tol:
            self.get_logger().info(
                f'wp {self.cur_wp_idx + 1}/{len(self.waypoints)} reached at '
                f'({self.x:.2f},{self.y:.2f})'
            )
            self.cur_wp_idx += 1
            self.best_dist_to_wp = float('inf')
            self.last_progress_t = self.get_clock().now()
            self._stop()
            return

        # Track progress; bail if no improvement for `stuck_timeout_s`.
        if dist < self.best_dist_to_wp - 0.05:
            self.best_dist_to_wp = dist
            self.last_progress_t = self.get_clock().now()
        else:
            stuck_for = (self.get_clock().now() - self.last_progress_t).nanoseconds * 1e-9
            if stuck_for > self.stuck_timeout_s:
                self.get_logger().warn(
                    f'wp {self.cur_wp_idx + 1}: stuck for {stuck_for:.1f}s '
                    f'(best={self.best_dist_to_wp:.2f}, cur={dist:.2f}). Skipping.'
                )
                self.cur_wp_idx += 1
                self.best_dist_to_wp = float('inf')
                self.last_progress_t = self.get_clock().now()
                self._stop()
                return

        # If pointing wrong way, rotate in place first.
        if abs(yaw_err) > self.yaw_tol:
            self._publish(0.0, self.max_w * (1 if yaw_err > 0 else -1))
            return

        # Drive forward with a gentle heading correction.
        v = min(self.max_v, 0.6 * dist + 0.05)
        w = max(-self.max_w, min(self.max_w, 1.5 * yaw_err))
        self._publish(v, w)


def main(argv=None):
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument('trajectory', type=Path,
                    help='YAML file with waypoints (or a name in '
                          'config/trajectories/).')
    p.add_argument('--max-runtime-s', type=float, default=240.0,
                    help='Wall-clock kill switch.')
    p.add_argument('--no-sim-time', action='store_true',
                    help='Disable use_sim_time (default: use ROS sim clock).')
    args = p.parse_args(argv)

    if not args.trajectory.exists():
        # try resolving as a package name
        try:
            from ament_index_python.packages import get_package_share_directory
            share = Path(get_package_share_directory('rove_sim_webots'))
            cand = share / 'config' / 'trajectories' / (
                args.trajectory.name if args.trajectory.name.endswith('.yaml')
                else f'{args.trajectory.name}.yaml')
            if cand.exists():
                args.trajectory = cand
        except Exception:
            pass
    traj = yaml.safe_load(args.trajectory.read_text())

    rclpy.init()
    node = WaypointDriver(traj, use_sim_time=not args.no_sim_time)
    t0 = time.monotonic()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() - t0 > args.max_runtime_s:
                node.get_logger().warn('max runtime exceeded; stopping.')
                break
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main() or 0)
