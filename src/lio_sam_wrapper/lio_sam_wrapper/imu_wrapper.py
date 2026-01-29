#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuWrapper(Node):
    def __init__(self):
        super().__init__('imu_wrapper')

        # Parameters
        self.declare_parameter('input_topic', '/imu_raw')
        self.declare_parameter('output_topic', '/imu')
        self.declare_parameter('accel_in_g', True)
        self.declare_parameter('gyro_in_deg', True)

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.accel_in_g = self.get_parameter('accel_in_g').get_parameter_value().bool_value
        self.gyro_in_deg = self.get_parameter('gyro_in_deg').get_parameter_value().bool_value

        self.sub = self.create_subscription(
            Imu,
            self.input_topic,
            self.imu_callback,
            rclpy.qos.qos_profile_sensor_data
        )

        self.pub = self.create_publisher(Imu, self.output_topic, 10)

        self.get_logger().info(
            f"IMU unit fixer running:\n"
            f"  input_topic:  {self.input_topic}\n"
            f"  output_topic: {self.output_topic}\n"
            f"  accel_in_g:   {self.accel_in_g}\n"
            f"  gyro_in_deg:  {self.gyro_in_deg}"
        )

    def imu_callback(self, msg: Imu):
        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance
        out.linear_acceleration = msg.linear_acceleration
        out.angular_velocity = msg.angular_velocity
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        out.angular_velocity_covariance = msg.angular_velocity_covariance

        # Convert acceleration
        if self.accel_in_g:
            G = 9.80665
            out.linear_acceleration.x *= G
            out.linear_acceleration.y *= G
            out.linear_acceleration.z *= G

        # Convert angular velocity
        if self.gyro_in_deg:
            DEG2RAD = math.pi / 180.0
            out.angular_velocity.x *= DEG2RAD
            out.angular_velocity.y *= DEG2RAD
            out.angular_velocity.z *= DEG2RAD

        self.pub.publish(out)


def main():
    rclpy.init()
    node = ImuWrapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
