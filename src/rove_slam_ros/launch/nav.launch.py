"""Full nav2 stack alongside rove_slam SLAM. Headless — no rviz.

Brings up:
  - rove_slam_node            (lidar SLAM, publishes /odom + map→odom→base_link)
  - nav2 controller_server     (DWB local planner)
  - nav2 planner_server        (NavFn global planner)
  - nav2 behavior_server       (spin, backup, drive_on_heading, wait)
  - nav2 bt_navigator          (behavior tree dispatcher)
  - nav2 waypoint_follower
  - nav2 velocity_smoother     (smooths /cmd_vel_nav → /cmd_vel)
  - lifecycle_manager_navigation (activates the above)
  - local + global costmap     (configured under controller_server / planner_server)

Send a goal once everything's up:
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
       "{pose: {header: {frame_id: 'map'},
                pose: {position: {x: 2.0, y: 0.0, z: 0.0},
                        orientation: {w: 1.0}}}}"

  ros2 topic echo /cmd_vel    # observe controller output
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

    lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),

        # 1) SLAM.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg, "launch", "slam.launch.py"])
            ),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "params_file": slam_params,
            }.items(),
        ),

        # 2) nav2 stack — minimal headless set.
        Node(package="nav2_controller", executable="controller_server",
             name="controller_server", output="screen",
             parameters=[nav_params,
                         {"use_sim_time": LaunchConfiguration("use_sim_time")}],
             remappings=[("cmd_vel", "cmd_vel_nav")]),
        Node(package="nav2_planner", executable="planner_server",
             name="planner_server", output="screen",
             parameters=[nav_params,
                         {"use_sim_time": LaunchConfiguration("use_sim_time")}]),
        Node(package="nav2_behaviors", executable="behavior_server",
             name="behavior_server", output="screen",
             parameters=[nav_params,
                         {"use_sim_time": LaunchConfiguration("use_sim_time")}]),
        Node(package="nav2_bt_navigator", executable="bt_navigator",
             name="bt_navigator", output="screen",
             parameters=[nav_params,
                         {"use_sim_time": LaunchConfiguration("use_sim_time")}]),
        Node(package="nav2_waypoint_follower", executable="waypoint_follower",
             name="waypoint_follower", output="screen",
             parameters=[nav_params,
                         {"use_sim_time": LaunchConfiguration("use_sim_time")}]),
        Node(package="nav2_velocity_smoother", executable="velocity_smoother",
             name="velocity_smoother", output="screen",
             parameters=[nav_params,
                         {"use_sim_time": LaunchConfiguration("use_sim_time")}],
             remappings=[("cmd_vel", "cmd_vel_nav"),
                          ("cmd_vel_smoothed", "cmd_vel")]),

        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[{
                 "use_sim_time": LaunchConfiguration("use_sim_time"),
                 "autostart": True,
                 "bond_timeout": 0.0,
                 "node_names": lifecycle_nodes,
             }]),
    ])
