"""Minimal RTAB-Map launcher for 3D LiDAR + IMU input (no camera).

Bypasses rtabmap_launch.launch.py because its `subscribe_rgbd:=false` arg
doesn't actually disable the rtabmap node's RGB exact-sync subscription —
which causes the node to wait forever for camera topics that never publish
when feeding it real bags. Same gotcha the tuner template solves; this is
the equivalent for the orchestrator's phase-2 RealEvaluator.

Required launch args:
    database_path   absolute path where the rtabmap.db will be written
Optional:
    frame_id        TF frame of the lidar  (default: livox_frame)
    lidar_topic     PointCloud2 topic      (default: /livox/lidar)
    imu_topic       Imu topic              (default: /livox/imu)
    use_sim_time    'true' to use /clock   (default: true — matches bag-replay)
    qos             rclcpp QoS depth       (default: 1)
    rtabmap_args    space-separated extra --Key value pairs to override params
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *_, **__):
    frame_id = LaunchConfiguration('frame_id').perform(context)
    lidar_topic = LaunchConfiguration('lidar_topic').perform(context)
    imu_topic = LaunchConfiguration('imu_topic').perform(context)
    db_path = LaunchConfiguration('database_path').perform(context)
    use_sim = LaunchConfiguration('use_sim_time').perform(context).lower() in ('true', '1')
    qos = int(LaunchConfiguration('qos').perform(context))
    extra_args_str = LaunchConfiguration('rtabmap_args').perform(context)

    # Pass-through CLI arguments for the rtabmap binary so callers can
    # override any RTAB-Map parameter (Icp/VoxelSize, Reg/Force3DoF, etc.).
    # Format on input: "--Icp/VoxelSize 0.05 --Reg/Force3DoF true"
    arguments = ['-d']  # always start fresh
    if extra_args_str.strip():
        arguments.extend(extra_args_str.split())

    shared = {
        'use_sim_time': use_sim,
        'frame_id': frame_id,
        'qos': qos,
        'approx_sync': False,        # exact sync — bag has matched stamps
        'wait_for_transform': 0.2,
    }

    icp_odom_params = {
        **shared,
        'odom_frame_id': 'icp_odom',
        'expected_update_rate': 50.0,
        'wait_imu_to_init': True,
        'publish_null_when_lost': True,
    }

    rtabmap_params = {
        **shared,
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'subscribe_rgbd': False,
        'subscribe_scan_cloud': True,
        'subscribe_odom_info': True,
        'odom_sensor_sync': True,
        'map_frame_id': 'map',
        'database_path': db_path,
    }

    remap_imu = [('imu', imu_topic)] if imu_topic else [('imu', 'imu_not_used')]

    return [
        Node(
            package='rtabmap_odom',
            executable='icp_odometry',
            name='icp_odometry',
            namespace='rtabmap',
            output='screen',
            parameters=[icp_odom_params],
            remappings=remap_imu + [('scan_cloud', lidar_topic), ('odom', 'odom')],
            arguments=arguments,
        ),
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            namespace='rtabmap',
            output='screen',
            parameters=[rtabmap_params],
            remappings=remap_imu + [
                ('scan_cloud', lidar_topic),
                ('odom', 'odom'),
            ],
            arguments=arguments,
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('database_path'),
        DeclareLaunchArgument('frame_id', default_value='livox_frame'),
        DeclareLaunchArgument('lidar_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('qos', default_value='1'),
        DeclareLaunchArgument('rtabmap_args', default_value=''),
        OpaqueFunction(function=_launch_setup),
    ])
