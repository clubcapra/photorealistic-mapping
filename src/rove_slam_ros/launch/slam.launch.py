"""SLAM-only launch. Subscribes /livox/lidar (+ /imu/data when present),
publishes TF + /odom + /cloud_obstacles. Useful as a building block; pair
with `bag_replay.launch.py` to drive it from a rosbag2."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_default = PathJoinSubstitution(
        [FindPackageShare("rove_slam_ros"), "config", "slam.yaml"]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=params_default,
            description="Path to rove_slam_node yaml params.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use /clock if running off a rosbag2 with --clock.",
        ),
        Node(
            package="rove_slam_ros",
            executable="rove_slam_node",
            name="rove_slam_node",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
        ),
    ])
