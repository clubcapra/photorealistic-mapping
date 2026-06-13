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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("rove_slam_ros")

    return LaunchDescription([
        DeclareLaunchArgument("bag",
            default_value="/home/iliana/bags/moving_extra_long_bag2"),
        DeclareLaunchArgument("rate", default_value="1.0"),
        DeclareLaunchArgument(
            "loop", default_value="false",
            description="Loop the bag indefinitely. Useful for short bags "
                        "(like rosbag2_test_camera_lidars at 27.5 s) where "
                        "the goal-send + cmd_vel watch otherwise outruns "
                        "the bag and TF goes stale.",
        ),
        DeclareLaunchArgument(
            "bag_lidar_topic", default_value="/livox/lidar",
            description="Topic the bag publishes lidar on. Remapped → "
                        "/livox/lidar (what SLAM subscribes to). For the "
                        "dual-lidar camera bag pass "
                        "bag_lidar_topic:=/livox/lidar_192_168_2_40 to use "
                        "the primary lidar.",
        ),

        # Bag replay. Wall time everywhere (use_sim_time=false). Remap
        # /tf + /tf_static so the bag's stale chain doesn't compete with
        # the live SLAM tree. Lidar remap is no-op when bag_lidar_topic
        # already equals /livox/lidar.
        OpaqueFunction(function=lambda ctx: [ExecuteProcess(
            cmd=(
                [
                    "ros2", "bag", "play",
                    LaunchConfiguration("bag").perform(ctx),
                    "--rate", LaunchConfiguration("rate").perform(ctx),
                ]
                + (["--loop"]
                   if LaunchConfiguration("loop").perform(ctx).lower() == "true"
                   else [])
                + [
                    "--remap",
                    "/tf:=/tf_bag_unused",
                    "/tf_static:=/tf_static_bag_unused",
                    LaunchConfiguration("bag_lidar_topic").perform(ctx) +
                        ":=/livox/lidar",
                ]
            ),
            output="screen",
        )]),

        # SLAM + nav2 via nav2_bringup's well-tested launch.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg, "launch", "nav_bringup.launch.py"])
            ),
            launch_arguments={"use_sim_time": "false"}.items(),
        ),
    ])
