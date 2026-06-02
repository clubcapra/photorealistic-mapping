"""One launch file to rule them all.

Default (no args) — live mode on the Rove:
    SLAM + full nav2 stack. Subscribes /livox/lidar, /imu/data; publishes
    TF / /odom / /cloud_obstacles, runs nav2 (controller, planner, BT,
    behaviors, recovery, smoother), accepts NavigateToPose goals.

  ros2 launch rove_slam_ros bringup.launch.py

Bag replay (headless smoke test):
  ros2 launch rove_slam_ros bringup.launch.py \\
       bag:=/home/iliana/bags/moving_extra_long_bag2

Drive the Rove (send /cmd_vel over UDP :9101 to the Jetson):
  ros2 launch rove_slam_ros bringup.launch.py \\
       drive:=true rove_host:=jetson.local

Disable bits:
  bringup.launch.py with_nav:=false             # SLAM only
  bringup.launch.py with_nav:=false bag:=<dir>  # SLAM + bag, no nav

Send a goal after it's up:
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \\
       '{pose: {header: {frame_id: new_map},
                pose: {position: {x: 2, y: 0}, orientation: {w: 1}}}}'
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _bag_action(context, *_, **__):
    """Only spawn the rosbag2 player if `bag:=<path>` was provided."""
    bag = LaunchConfiguration("bag").perform(context)
    if not bag:
        return []
    rate = LaunchConfiguration("rate").perform(context)
    return [
        ExecuteProcess(
            cmd=[
                "ros2", "bag", "play", bag,
                "--rate", rate,
                # Remap the bag's stale `/tf` and `/tf_static` so they don't
                # compete with the live SLAM TF tree.
                "--remap", "/tf:=/tf_bag_unused", "/tf_static:=/tf_static_bag_unused",
            ],
            output="screen",
        )
    ]


def generate_launch_description():
    pkg = FindPackageShare("rove_slam_ros")
    slam_launch = PathJoinSubstitution([pkg, "launch", "slam.launch.py"])
    nav_launch = PathJoinSubstitution([pkg, "launch", "nav_bringup.launch.py"])

    return LaunchDescription([
        # ─── args ─────────────────────────────────────────────────────
        DeclareLaunchArgument(
            "bag", default_value="",
            description="rosbag2 directory to replay. Empty = live lidar mode.",
        ),
        DeclareLaunchArgument(
            "rate", default_value="1.0",
            description="Bag-play rate (1.0 = real-time). Ignored when bag is empty.",
        ),
        DeclareLaunchArgument(
            "with_nav", default_value="true",
            description="Run the full nav2 stack alongside SLAM.",
        ),
        DeclareLaunchArgument(
            "drive", default_value="false",
            description="Bridge /cmd_vel → Rove UDP RoveControl protobuf. "
                        "Only enable on the live robot — does not gate by mode.",
        ),
        DeclareLaunchArgument(
            "rove_host", default_value="127.0.0.1",
            description="Destination host for the Rove drive bridge.",
        ),
        DeclareLaunchArgument(
            "rove_port", default_value="9101",
            description="Destination UDP port on the Rove.",
        ),
        DeclareLaunchArgument(
            "viewer", default_value="false",
            description="Spawn the rerun live 3D viewer (subscribes to /tf, /odom, "
                        "/cloud_obstacles). Needs `pip install rerun-sdk`.",
        ),
        DeclareLaunchArgument(
            "viewer_raw", default_value="false",
            description="Also stream /livox/lidar to the rerun viewer (heavy).",
        ),
        DeclareLaunchArgument(
            "rviz", default_value="false",
            description="Open rviz2 with the rove_slam config alongside everything "
                        "else. Pre-wired to TF, /cloud_obstacles, /odom, /plan, "
                        "/global_costmap, /local_costmap.",
        ),
        DeclareLaunchArgument(
            "mesh_method", default_value="poisson",
            description="Mesh reconstruction backend used by the mesh_builder "
                        "node. One of: poisson | bpa | tsdf | nvblox. Built on "
                        "demand via the ~/build_mesh service, or at shutdown "
                        "when build_mesh_on_shutdown:=true. Compare with the "
                        "build_mesh.py CLI tool of the same name.",
        ),
        DeclareLaunchArgument(
            "build_mesh_on_shutdown", default_value="true",
            description="When the mesh_builder node receives SIGINT/SIGTERM, "
                        "auto-trigger the build before exiting (recommended for "
                        "bag replay smoke tests).",
        ),
        DeclareLaunchArgument(
            "mesh_output_dir", default_value="/tmp/rove_slam_mesh",
            description="Where mesh_builder writes trajectory.tum / dense.pcd "
                        "/ mesh_<method>.ply / build.log.",
        ),
        DeclareLaunchArgument(
            "colorize_mesh", default_value="false",
            description="After a mesh build, run scripts/color_mesh.py to "
                        "project the bag's camera images onto the mesh, "
                        "producing mesh_<method>_colored.ply. Requires bag + "
                        "urdf_path + (optional) cam_intrinsics_path.",
        ),
        DeclareLaunchArgument(
            "urdf_path", default_value="",
            description="Path to the URDF used by color_mesh.py to compose "
                        "cam_*_optical_frame ← base_link transforms. "
                        "Usually src/rove_description/urdf/rove_standard.urdf.",
        ),
        DeclareLaunchArgument(
            "cam_intrinsics_path", default_value="",
            description="Optional YAML with per-camera intrinsics. "
                        "Defaults to src/rove_slam_ros/config/cam_intrinsics.yaml "
                        "(PLACEHOLDER 90°-HFOV values) if left empty.",
        ),

        # ─── bag replay (conditional) ─────────────────────────────────
        OpaqueFunction(function=_bag_action),

        # ─── SLAM ─────────────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={"use_sim_time": "false"}.items(),
        ),

        # ─── nav2 (conditional) ───────────────────────────────────────
        GroupAction(
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav_launch),
                    launch_arguments={"use_sim_time": "false"}.items(),
                ),
            ],
            condition=IfCondition(LaunchConfiguration("with_nav")),
        ),

        # ─── /cmd_vel → Rove drive bridge (conditional) ──────────────
        Node(
            package="rove_slam_ros",
            executable="cmd_vel_to_rove.py",
            name="cmd_vel_to_rove",
            output="screen",
            parameters=[{
                "host": LaunchConfiguration("rove_host"),
                "port": LaunchConfiguration("rove_port"),
            }],
            condition=IfCondition(LaunchConfiguration("drive")),
        ),

        # ─── live rerun 3D viewer (conditional) ──────────────────────
        Node(
            package="rove_slam_ros",
            executable="rerun_live.py",
            name="rerun_live",
            output="screen",
            parameters=[{
                "spawn": True,
                "raw": LaunchConfiguration("viewer_raw"),
            }],
            condition=IfCondition(LaunchConfiguration("viewer")),
        ),

        # ─── rviz2 fallback viewer (conditional) ─────────────────────
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=[
                "-d",
                PathJoinSubstitution([pkg, "config", "rove_slam.rviz"]),
            ],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),

        # ─── mesh_builder (always on; triggered via service / shutdown) ──
        Node(
            package="rove_slam_ros",
            executable="mesh_builder.py",
            name="mesh_builder",
            output="screen",
            parameters=[{
                "method": LaunchConfiguration("mesh_method"),
                "output_dir": LaunchConfiguration("mesh_output_dir"),
                "build_on_shutdown": LaunchConfiguration("build_mesh_on_shutdown"),
                "urdf_extrinsic": True,
                "colorize": LaunchConfiguration("colorize_mesh"),
                "bag_path": LaunchConfiguration("bag"),
                "urdf_path": LaunchConfiguration("urdf_path"),
                "cam_intrinsics_path": LaunchConfiguration("cam_intrinsics_path"),
            }],
        ),
    ])
