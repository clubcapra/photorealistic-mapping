
import os
from ament_index_python.packages import get_package_share_directory  # FIX 1: was missing .packages
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro  # add this at the top of the launch file

 
def generate_launch_description():
 
    # ── Livox Mid360 ────────────────────────────────────────────────────────
    # FIX 2: Use msg_MID360_launch (publishes PointCloud2) NOT rviz_MID360_launch
    #         rviz_MID360_launch opens RViz AND uses CustomMsg format — wrong for RTABMap
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('livox_ros_driver2'),
            'launch_ROS2',
            'msg_MID360_launch.py'         # publishes /livox/lidar as PointCloud2
        ))
    )
 
    # ── VectorNav VN300 ─────────────────────────────────────────────────────
    vectornav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rove_color_mapping'),
            'launch',
            'vectornav.launch.py'
        )),
        launch_arguments={
            "xfer_format": "0",
        }.items()
    )
 
    # ── Robot State Publisher (publishes TF from your XACRO) ────────────────
    # FIX 3: You need this so RTABMap can find base_link → livox_frame → imu_link
    urdf_file = os.path.join(
        get_package_share_directory('rove_color_mapping'),
        'urdf',
        'sensor_mount.urdf.xacro'
    )

    robot_description = xacro.process_file(urdf_file).toxml()  # expands all variables

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description  # pass raw URDF string
            # NOTE: if you use xacro macros, do:
            #   import xacro
            #   robot_description = xacro.process_file(urdf_file).toxml()
        }]
    )

    # ── RTABMap odometry node (separate from the mapping node) ───────────────
    # This generates odom → base_link from the LiDAR scan.
    # Splitting odometry and mapping into two nodes is the correct RTABMap
    # architecture for LiDAR-only — the mapping node then consumes /odom.
    rtabmap_odom_node = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='rtabmap_odom',
        output='screen',
        parameters=[{
            'frame_id':       'base_link',
            'odom_frame_id':  'odom',

            'subscribe_lidar': True,
            'subscribe_imu':   True,

            'approx_sync':              True,
            'approx_sync_max_interval': 0.02,
            'topic_queue_size':         30,
            'sync_queue_size':          30,

            'wait_for_transform': 0.5,
            'tf_tolerance':       0.2,

            # IMU upside-down correction
            'imu_local_transform': '0 0 0 3.14159 0 0',
            'wait_imu_to_init':    True,

            # ICP odometry params
            'Icp/PointToPlane':              'true',
            'Icp/Iterations':                '10',
            'Icp/VoxelSize':                 '0.1',
            'Icp/MaxCorrespondenceDistance': '1.0',
            'Odom/Strategy':                 '0',
            'OdomF2M/ScanSubtractRadius':    '0.1',
        }],
        remappings=[
            ('scan_cloud', '/livox/lidar'),
            ('imu',        '/vectornav/imu'),
        ]
    )
 
    # ── RTABMap ─────────────────────────────────────────────────────────────
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'subscribe_depth':  False,
            'subscribe_lidar':  True,
            'subscribe_imu':    True,
            'use_action_for_goal': True,
            'subscribe_rgb': False,
            'RGBD/Enabled': 'false',
            
            # Frames — must match your XACRO link names
            'frame_id':      'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id':  'map',
 
            # FIX 4: approx_sync needed when LiDAR and IMU have different rates
            'approx_sync':   True,
            'approx_sync_max_interval': 0.01,   # 10 ms tolerance
 
            # LiDAR ICP params
            'Icp/PointToPlane':              'true',
            'Icp/Iterations':                '10',
            'Icp/VoxelSize':                 '0.1',
            'Icp/MaxCorrespondenceDistance': '1.0',
 
            # Memory
            'Mem/IncrementalMemory':  'true',
            'Mem/InitWMWithAllNodes': 'true',
 
            # IMU integration
            'wait_imu_to_init':        True,    # FIX 5: bool not string
            'Odom/Strategy':           '0',
            'OdomF2M/ScanSubtractRadius': '0.1',
 
            # FIX 6: IMU is upside-down — tell RTABMap to expect inverted gravity
            # This applies a 180° roll to the IMU data to account for the mount
            'imu_local_transform':  '0 0 0 3.14159 0 0',  # x y z roll pitch yaw
 
            # Loop closure / registration
            'RGBD/ProximityBySpace': 'true',
            'Reg/Strategy':          '1',       # ICP
        }],
        remappings=[
            # FIX 7: both remappings point to the same /livox/lidar topic — be consistent.
            # scan_cloud_topic param AND the remapping must agree on ONE topic name.
            # Since msg_MID360_launch publishes PointCloud2 on /livox/lidar, use that directly.
            ('scan_cloud', '/livox/lidar'),
            # ('imu',        '/vectornav/imu'),
        ]
    )
 
    # ── RTABMap Viz ──────────────────────────────────────────────────────────
    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            'subscribe_depth':  False,
            'subscribe_rgb':    False,
            'subscribe_lidar':  True,
            'RGBD/Enabled':     'false',
            'frame_id':         'base_link',
            'approx_sync':      True,
            'topic_queue_size': 30,
            'sync_queue_size':  30,
        }],
        remappings=[
            ('scan_cloud', '/livox/lidar'),
            ('imu',        '/vectornav/imu'),
        ]
    )
 
    return LaunchDescription([
        livox_launch,
        vectornav_launch,
        robot_state_publisher,   # FIX 3: must come before RTABMap
        rtabmap_node,
        rtabmap_viz_node,
    ])