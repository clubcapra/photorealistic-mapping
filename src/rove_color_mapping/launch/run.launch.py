import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('rove_color_mapping')

    # ── Livox Mid360 ─────────────────────────────────────────────────────────
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('livox_ros_driver2'),
            'launch_ROS2',
            'msg_MID360_launch.py'
        )),
        launch_arguments={'frame_id': 'livox_frame'}.items()
    )

    # ── VectorNav VN300 ──────────────────────────────────────────────────────
    vectornav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg, 'launch', 'vectornav.launch.py'
        ))
    )

    # ── Robot State Publisher ────────────────────────────────────────────────
    urdf_file = os.path.join(pkg, 'urdf', 'sensor_mount.urdf.xacro')
    robot_description = xacro.process_file(urdf_file).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 50.0,
        }]
    )

    # ── ICP Odometry ─────────────────────────────────────────────────────────
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
            'qos_imu':             1,

            # ── ICP params tuned for Livox Mid360 ──────────────────────────
            'Icp/PointToPlane':              'true',
            'Icp/PointToPlaneK':             '20',
            'Icp/PointToPlaneRadius':        '0',
            'Icp/Iterations':                '30',    # more iterations = more robust
            'Icp/VoxelSize':                 '0.2',   # slightly coarser for speed
            'Icp/DownsamplingStep':          '1',
            'Icp/OutlierRatio':              '0.85',  # allow more outliers
            'Icp/CorrespondenceRatio':       '0.01',  # very permissive — Livox is sparse

            # FIX: loosen motion limits — these were causing the resets
            'Icp/MaxTranslation':            '3.0',   # was implicitly 0.2 — now 3m
            'Icp/MaxRotation':               '3.14',  # allow up to 180° rotation
            'Icp/MaxCorrespondenceDistance': '0.5',   # tighter correspondence

            # Odometry
            'Odom/Strategy':              '0',        # Frame-to-Map
            'Odom/ResetCountdown':        '0',        # don't auto-reset
            'Odom/GuessMotion':           'true',     # use IMU to seed ICP guess
            'OdomF2M/ScanSubtractRadius': '0.1',
            'OdomF2M/MaxSize':            '20000',    # larger local map
            'OdomF2M/ScanMaxSize':        '20000',
        }],
        remappings=[
            ('scan_cloud', '/livox/lidar'),
            ('imu',        '/vectornav/imu'),
        ]
    )

    # ── RTABMap SLAM ─────────────────────────────────────────────────────────
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'subscribe_depth':        False,
            'subscribe_rgb':          False,
            'subscribe_stereo':       False,
            'subscribe_rgbd':         False,
            'subscribe_sensor_data':  False,
            'subscribe_scan':         False,
            'subscribe_lidar':        True,
            'subscribe_imu':          True,
            'RGBD/Enabled':           'false',
            'use_action_for_goal':    True,

            'frame_id':      'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id':  'map',

            'wait_for_transform': 0.5,
            'tf_tolerance':       0.2,

            'approx_sync':              True,
            'approx_sync_max_interval': 0.02,
            'topic_queue_size':         30,
            'sync_queue_size':          30,

            'database_path': os.path.join(
                os.path.expanduser('~'), '.ros', 'rtabmap_lidar.db'
            ),

            # ICP for loop closure — can be stricter than odometry
            'Icp/PointToPlane':              'true',
            'Icp/Iterations':                '30',
            'Icp/VoxelSize':                 '0.2',
            'Icp/MaxCorrespondenceDistance': '0.5',
            'Icp/CorrespondenceRatio':       '0.01',
            'Icp/OutlierRatio':              '0.85',
            'Icp/MaxTranslation':            '3.0',

            'Mem/IncrementalMemory':  'true',
            'Mem/InitWMWithAllNodes': 'true',

            'RGBD/ProximityBySpace': 'true',
            'Reg/Strategy':          '1',
        }],
        remappings=[
            ('scan_cloud', '/livox/lidar'),
            ('imu',        '/vectornav/imu'),
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
        robot_state_publisher,
        rtabmap_odom_node,
        rtabmap_node,
        rtabmap_viz_node,
    ])
