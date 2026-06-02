import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # ── Livox Mid360 (lidar) ───────────────────────────────────────────────────────────────
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('livox_ros_driver2'),
            'launch_ROS2',
            'msg_MID360_launch.py'
        )),
        launch_arguments={'frame_id': 'livox_frame'}.items()
    )

    # ── VectorNav VN300 (IMU) ────────────────────────────────────────────────────────────
    vectornav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('vectornav_udp_bridge'),
            'launch', 'run.launch.py'
        ))
    )

    # ── Nav2 (costmaps)───────────────────────────────────────────────────────────────────────
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rove_color_mapping'),
            'launch', 'nav.launch.py'
        ))
    )

    # ── Robot State Publisher (Test Rig)────────────────────────────────────────────
    # urdf_file = os.path.join(pkg, 'urdf', 'sensor_mount.urdf.xacro')
    # robot_description = xacro.process_file(urdf_file).toxml()

    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='screen',
    #     parameters=[{
    #         'robot_description': robot_description,
    #         'publish_frequency': 50.0,
    #     }]
    # )

    # ── Robot State Publisher (Rove)────────────────────────────────────────────────
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rove_description'),
            'launch',
            'launch.py'
        ))
    )

    # ── RTABMap (mapping) ────────────────────────────────────────────────────────────────────
    # Pass our specific topics and frame into it directly
    rtabmap_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rove_color_mapping'),
            'launch', 'rtabmap.launch.py'
        ))
    )

    # ── Gscam2 (camera) ────────────────────────────────────────────────────────────────────
    # Pass our specific topics and frame into it directly
    gscam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rove_color_mapping'),
            'launch', 'gscam.launch.py'
        ))
    )
    return LaunchDescription([
        livox_launch,
        # vectornav_launch,
        robot_state_publisher,
        # rtabmap_lidar_launch,
        gscam_launch,
        # nav2_launch
    ])