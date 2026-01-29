from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lio_sam_wrapper',
            executable='imu_wrapper',
            name='imu_wrapper',
            output='screen',
            parameters=[{
                'input_topic': '/livox/imu',
                'output_topic': '/imu',
                'accel_in_g': True,
                'gyro_in_deg': True,
            }]
        )
    ])
