"""
launch/vectornav.launch.py
--------------------------
Launch the VectorNav UDP bridge node on the Jetson.

Usage:
  ros2 launch vectornav_udp_bridge vectornav.launch.py pi_ip:=192.168.2.153
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pi_ip',    default_value='192.168.2.2',
                              description='IP address of the Capra Rove Pi'),
        DeclareLaunchArgument('udp_port', default_value='5000',
                              description='VectorNav data UDP port'),
        DeclareLaunchArgument('imu_frame', default_value='imu_link',
                              description='TF frame for IMU messages'),
        DeclareLaunchArgument('gps_frame', default_value='gps_link',
                              description='TF frame for NavSatFix messages'),
        DeclareLaunchArgument('subscribe_interval', default_value='5.0',
                              description='Seconds between re-sending Subscribe'),

        Node(
            package='vectornav_udp_bridge',
            executable='vectornav_udp_node',
            name='vectornav_udp_node',
            output='screen',
            parameters=[{
                'pi_ip':               LaunchConfiguration('pi_ip'),
                'udp_port':            ParameterValue(LaunchConfiguration('udp_port'), value_type=int),
                'imu_frame':           LaunchConfiguration('imu_frame'),
                'gps_frame':           LaunchConfiguration('gps_frame'),
                'subscribe_interval':  ParameterValue(LaunchConfiguration('subscribe_interval'), value_type=float),
            }],
        ),
    ])
