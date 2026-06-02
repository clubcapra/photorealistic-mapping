"""Launch isaac_ros_nvblox alongside SLAM for live CUDA-native TSDF
fusion with RGB-D color from the cardinal cameras.

  ros2 launch rove_slam_ros nvblox.launch.py

Requires the `nvblox_ros` package on the system. This will be present
on Jetson + isaac_ros installs, and absent on plain Humble x86. The
launch fails fast with a clear error in the latter case (so the
fallback `mesh_method:=tsdf` path can be used instead).

What it does:
  * Brings up `nvblox_node` with our voxel + integration params.
  * Remaps `/livox/lidar` → the node's lidar input topic so it
    consumes our SLAM-frame pointcloud directly.
  * Remaps each `/cam_<dir>/image_raw` + `/cam_<dir>/camera_info`
    to one of nvblox's color-cam slots (up to 4 supported).
  * Reads the TF tree published by SLAM (`map → odom → base_link`)
    to anchor every integration step.

What it does NOT do:
  * It does NOT republish lidar from `/livox/lidar_192_168_2_*` —
    use the same remap as `bag_replay.launch.py` upstream.
  * It does NOT bring up SLAM. Run this AFTER `slam.launch.py` (or
    via `bringup.launch.py mesh_method:=nvblox`).

Save the mesh on demand with:
  ros2 service call /nvblox_node/save_ply nvblox_msgs/srv/FilePath \\
      "{file_path: '/tmp/rove_slam_mesh/mesh_nvblox.ply'}"
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_nvblox_node(context, *_args, **_kwargs):
    # Late-resolve so we get a friendly error if nvblox_ros is missing.
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory("nvblox_ros")
    except Exception as e:
        raise RuntimeError(
            "nvblox_ros package not found on this system. Install "
            "isaac_ros_nvblox (Jetson / x86 with CUDA), or pick a "
            "different mesh_method (poisson | tsdf) which run on CPU. "
            f"underlying error: {e}"
        )

    voxel = float(LaunchConfiguration("voxel").perform(context))
    max_dist = float(LaunchConfiguration("max_integration_distance_m").perform(context))
    global_frame = LaunchConfiguration("global_frame").perform(context)

    return [
        Node(
            package="nvblox_ros",
            executable="nvblox_node",
            name="nvblox_node",
            output="screen",
            parameters=[{
                # Frames
                "global_frame": global_frame,            # map (SLAM-published)
                "pose_frame": "base_link",

                # Integration
                "voxel_size": voxel,
                "max_integration_distance_m": max_dist,
                "use_lidar": True,
                "use_depth": False,                       # we only have lidar+RGB
                "use_color": True,

                # Lidar params (Livox MID-360-ish)
                "lidar_width": 1024,
                "lidar_height": 64,
                "lidar_min_valid_range_m": 1.0,
                "lidar_max_valid_range_m": 60.0,

                # Mesh streaming
                "esdf_2d": False,
                "esdf_distance_slice": False,
                "mesh": True,
                "compute_esdf": False,

                # ROS-time (wall) since SLAM publishes wall-time TF
                "use_sim_time": False,
            }],
            remappings=[
                # Lidar → nvblox's lidar input topic
                ("pointcloud", "/livox/lidar"),

                # 4 color cameras + camera_info. nvblox supports a
                # variable number of color inputs via the `color/N/`
                # pattern — wire the cardinals here.
                ("color/0/image", "/cam_north/image_raw"),
                ("color/0/camera_info", "/cam_north/camera_info"),
                ("color/1/image", "/cam_east/image_raw"),
                ("color/1/camera_info", "/cam_east/camera_info"),
                ("color/2/image", "/cam_south/image_raw"),
                ("color/2/camera_info", "/cam_south/camera_info"),
                ("color/3/image", "/cam_west/image_raw"),
                ("color/3/camera_info", "/cam_west/camera_info"),
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "voxel", default_value="0.05",
            description="TSDF voxel size (m). 0.05 = 5cm typical for indoor "
                        "scans. nvblox handles much finer voxels than CPU TSDF "
                        "thanks to GPU memory."
        ),
        DeclareLaunchArgument(
            "max_integration_distance_m", default_value="15.0",
            description="Reject lidar returns beyond this. Keep at or below "
                        "the lidar's reliable indoor range."
        ),
        DeclareLaunchArgument(
            "global_frame", default_value="map",
            description="World frame published by SLAM (rove_slam_node "
                        "defaults to `map`)."
        ),
        OpaqueFunction(function=_build_nvblox_node),
    ])
