"""SLAM + nav2 local costmap, headless. The full nav2 stack is a lot of
moving parts; this launch keeps the minimum needed to verify that SLAM
output feeds a working nav2 costmap (which is the integration smoke test).

  ros2 launch rove_slam_ros slam_nav.launch.py
  # then in another shell:
  ros2 topic echo /costmap/costmap   # should print a frame
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("rove_slam_ros")
    slam_params = PathJoinSubstitution([pkg, "config", "slam.yaml"])
    costmap_params = PathJoinSubstitution([pkg, "config", "nav2_costmap.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="Use /clock from rosbag2 --clock (set true for replay).",
        ),

        # SLAM node.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg, "launch", "slam.launch.py"])
            ),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "params_file": slam_params,
            }.items(),
        ),

        # nav2 local costmap. Lifecycle-managed by `lifecycle_manager_costmap`
        # below — the manager calls configure() + activate() on autostart.
        Node(
            package="nav2_costmap_2d",
            executable="nav2_costmap_2d",
            name="costmap",
            output="screen",
            parameters=[
                costmap_params,
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_costmap",
            output="screen",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "autostart": True,
                "bond_timeout": 0.0,         # don't enforce a bond — costmap
                                             # has been observed to time out
                                             # at "Configuring" on this env
                                             # without this. The bond is only
                                             # used by the manager to detect
                                             # node crashes; we're OK without.
                "node_names": ["costmap/costmap"],
            }],
        ),
    ])
