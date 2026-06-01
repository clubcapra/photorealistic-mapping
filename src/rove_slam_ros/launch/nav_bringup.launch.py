"""Bring up the full nav2 stack via nav2_bringup/navigation_launch.py.

`nav.launch.py` was a hand-rolled stack — clean for reading but hits
lifecycle-manager activation races with our config. This alternative just
delegates to nav2_bringup's well-tested launch, parameterised by our
nav2_full.yaml.

Use this for headless nav smoke tests; use nav.launch.py for debugging.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("rove_slam_ros")
    nav_params = PathJoinSubstitution([pkg, "config", "nav2_full.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),

        # 1) SLAM (publishes map → odom → base_link + /cloud_obstacles).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg, "launch", "slam.launch.py"])
            ),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }.items(),
        ),

        # 2) nav2 stack via the standard launch — well-tested timing.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("nav2_bringup"),
                    "launch", "navigation_launch.py",
                ])
            ),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "params_file": nav_params,
                "autostart": "true",
                "use_composition": "True",
            }.items(),
        ),
    ])
