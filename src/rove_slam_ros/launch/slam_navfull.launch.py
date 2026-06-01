"""SLAM + minimum-viable full nav2 stack (planner, controller, BT navigator,
behaviors, local/global costmaps). Headless. Uses our SLAM TF chain
(`new_map → odom → base_link`) as the localization source — no AMCL or
static map needed.

After launching, send a goal:
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \\
        '{pose: {header: {frame_id: new_map}, pose: {position: {x: 2, y: 0}, orientation: {w: 1}}}}'
and watch /cmd_vel:
    ros2 topic echo /cmd_vel
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
    nav_params = PathJoinSubstitution([pkg, "config", "nav2_full.yaml"])

    use_sim_time = LaunchConfiguration("use_sim_time")
    common_params = [nav_params, {"use_sim_time": use_sim_time}]

    # nav2 lifecycle-managed nodes.
    nav2_nodes = ["controller_server", "planner_server",
                  "behavior_server", "bt_navigator", "velocity_smoother"]

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="Use /clock from rosbag2 (true for replay).",
        ),

        # SLAM.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg, "launch", "slam.launch.py"])
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "params_file": slam_params,
            }.items(),
        ),

        # Local + global costmaps.
        Node(
            package="nav2_costmap_2d", executable="nav2_costmap_2d",
            name="local_costmap", output="screen", parameters=common_params,
        ),
        Node(
            package="nav2_costmap_2d", executable="nav2_costmap_2d",
            name="global_costmap", output="screen", parameters=common_params,
        ),

        # nav2 stack.
        Node(package="nav2_controller", executable="controller_server",
             name="controller_server", output="screen", parameters=common_params),
        Node(package="nav2_planner", executable="planner_server",
             name="planner_server", output="screen", parameters=common_params),
        Node(package="nav2_behaviors", executable="behavior_server",
             name="behavior_server", output="screen", parameters=common_params),
        Node(package="nav2_bt_navigator", executable="bt_navigator",
             name="bt_navigator", output="screen", parameters=common_params),
        Node(package="nav2_velocity_smoother", executable="velocity_smoother",
             name="velocity_smoother", output="screen", parameters=common_params),

        # Single lifecycle manager for everything (autostart). bond_timeout 0
        # because nav2's bond is flaky under heavy startup load and we don't
        # need crash detection for a smoke test.
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="lifecycle_manager_nav", output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": True,
                "bond_timeout": 0.0,
                "node_names": ["local_costmap/local_costmap",
                                "global_costmap/global_costmap",
                                *nav2_nodes],
            }],
        ),
    ])
