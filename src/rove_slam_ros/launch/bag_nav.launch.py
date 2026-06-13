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

        # Bag replay — NO --clock. The SLAM node restamps every message
        # with `get_clock()->now()` (wall time), so the whole graph runs
        # off real time and TF buffers stay consistent. With --clock the
        # bag's stale stamps + the launch-up race produce relentless
        # TF_OLD_DATA in nav2.
        # Remap /tf and /tf_static so the bag's old chain doesn't compete
        # with the live SLAM tree.
        ExecuteProcess(
            cmd=[
                "ros2", "bag", "play",
                LaunchConfiguration("bag"),
                "--rate", LaunchConfiguration("rate"),
                "--remap",
                "/tf:=/tf_bag_unused",
                "/tf_static:=/tf_static_bag_unused",
            ],
            output="screen",
        ),

        # SLAM + full nav2 stack — wall time everywhere (use_sim_time off).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg, "launch", "nav.launch.py"])
            ),
            launch_arguments={"use_sim_time": "false"}.items(),
        ),
    ])
