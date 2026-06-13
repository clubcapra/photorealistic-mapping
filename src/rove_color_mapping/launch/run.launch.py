import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)

    # ── Livox Mid360 (lidar) ───────────────────────────────────────────────────────────────
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('livox_ros_driver2'),
            'launch_ROS2',
            'msg_MID360_launch.py'
        )),
        launch_arguments={'frame_id': 'livox_frame', 'use_sim_time': use_sim_time}.items()
    )

    lidar_merger = Node(
        package='rove_color_mapping',
        executable='lidar_merger',
        name='lidar_merger',
        parameters=[{
            'topic_1': '/livox/lidar_192_168_2_40',
            'topic_2': '/livox/lidar_192_168_2_41',
            'output_topic': '/livox/lidar',
            'output_frame': 'livox_frame',
        }]
    )


    # ── VectorNav VN300 (IMU) ────────────────────────────────────────────────────────────
    vectornav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('vectornav_udp_bridge'),
            'launch', 'run.launch.py'
        )),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    core_stabilized_dummy = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='core_stabilized_dummy',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'Core', 'Core_stabilized']
    )  


    # ── Nav2 (costmaps)───────────────────────────────────────────────────────────────────────
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rove_color_mapping'),
            'launch', 'nav.launch.py'
        )),
        launch_arguments={'use_sim_time': use_sim_time}.items()
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
        )),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # ── RTABMap (mapping) ────────────────────────────────────────────────────────────────────
    # Pass our specific topics and frame into it directly
    rtabmap_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rove_color_mapping'),
            'launch', 'rtabmap.launch.py'
        )),
        launch_arguments={'use_sim_time': use_sim_time}.items()
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
        lidar_merger,
        # vectornav_launch,
        core_stabilized_dummy, #TODO if vectornav_launch is not commented, comment this line
        robot_state_publisher,
        rtabmap_lidar_launch,
        gscam_launch,
        # nav2_launch
    ])