"""Replay a rosbag2 + SLAM + costmap headless. Useful for offline testing
and CI smoke tests — no real lidar required.

  ros2 launch rove_slam_ros bag_replay.launch.py bag:=/home/iliana/bags/moving_extra_long_bag2
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("rove_slam_ros")

    return LaunchDescription([
        DeclareLaunchArgument(
            "bag", default_value="/home/iliana/bags/moving_extra_long_bag2",
            description="Path to rosbag2 directory.",
        ),
        DeclareLaunchArgument(
            "rate", default_value="1.0",
            description="Replay rate (1.0 = real-time).",
        ),
        DeclareLaunchArgument(
            "with_nav", default_value="true",
            description="Bring up the nav2 local costmap stack alongside SLAM.",
        ),

        # Replay process. --clock so use_sim_time downstream sees the bag time.
        # Remap the bag's /tf and /tf_static into dead-end topics — our SLAM
        # node publishes the live tree itself, and the bag's old (6-month-
        # stale) chain rooted at `new_map` would compete with the live tree
        # and crash the costmap with "two unconnected trees".
        # (Humble's `ros2 bag play` has no --exclude flag; remap is the
        # easiest equivalent. `--topics` allow-listing is the alternative
        # but requires enumerating every topic we want to keep.)
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

        # SLAM (+ costmap, when with_nav:=true).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg, "launch", "slam_nav.launch.py"])
            ),
            launch_arguments={"use_sim_time": "true"}.items(),
        ),
    ])
