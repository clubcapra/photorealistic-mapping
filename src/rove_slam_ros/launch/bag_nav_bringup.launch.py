"""Bag replay + SLAM + full nav2 (via nav2_bringup). Headless smoke test.

The single command that drives the headless integration test:

  ros2 launch rove_slam_ros bag_nav_bringup.launch.py \
       bag:=/home/iliana/bags/moving_extra_long_bag2

Then in another shell, send a goal:

  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \\
       "{pose: {header: {frame_id: 'new_map'},
                pose: {position: {x: 2.0, y: 0.0, z: 0.0},
                        orientation: {w: 1.0}}}}"

  ros2 topic echo /cmd_vel
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

        # Bag replay. Wall time everywhere (use_sim_time=false). Remap
        # /tf + /tf_static so the bag's stale chain doesn't compete with
        # the live SLAM tree.
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

        # SLAM + nav2 via nav2_bringup's well-tested launch.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg, "launch", "nav_bringup.launch.py"])
            ),
            launch_arguments={"use_sim_time": "false"}.items(),
        ),
    ])
