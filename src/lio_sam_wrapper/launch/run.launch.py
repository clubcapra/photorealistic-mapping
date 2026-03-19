import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='lio_sam_wrapper',
        #     executable='imu_wrapper',
        #     name='imu_wrapper',
        #     output='screen',
        #     parameters=[{
        #         'input_topic': '/livox/imu',
        #         'output_topic': '/imu',
        #         'accel_in_g': True,
        #         'gyro_in_deg': True,
        #     }]
        # ),
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
                        get_package_share_directory('livox_ros_driver2'),
                        'launch_ROS2/rviz_MID360_launch.py'))),
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
                        get_package_share_directory('lio_sam_wrapper'),
                        'launch/vectornav.launch.py'))),
        Node(
            package='lio_sam_wrapper',
            executable='lidar_wrapper',
            name='lidar_wrapper',
            output='screen',
        )
    ])
