"""Replay a rosbag2 + run full nav2 stack on top of rove_slam.

The smoke-test scenario for the headless nav integration: feed lidar from
a recorded bag, let SLAM publish TF + /cloud_obstacles, fire up the full
nav2 stack, send a NavigateToPose goal, watch /cmd_vel.

  ros2 launch rove_slam_ros bag_nav.launch.py bag:=/home/iliana/bags/moving_extra_long_bag2
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("rove_slam_ros")

    return LaunchDescription([
        DeclareLaunchArgument("bag",
            default_value="/home/iliana/bags/moving_extra_long_bag2"),
        DeclareLaunchArgument("rate", default_value="1.0"),

        # Bag replay — remap /tf so the bag's old chain doesn't compete.
        ExecuteProcess(
            cmd=[
                "ros2", "bag", "play",
                LaunchConfiguration("bag"),
                "--clock",
                "--rate", LaunchConfiguration("rate"),
                "--remap",
                "/tf:=/tf_bag_unused",
                "/tf_static:=/tf_static_bag_unused",
            ],
            output="screen",
        ),

        # SLAM + full nav2 stack, sim_time on so it tracks /clock.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg, "launch", "nav.launch.py"])
            ),
            launch_arguments={"use_sim_time": "true"}.items(),
        ),
    ])
