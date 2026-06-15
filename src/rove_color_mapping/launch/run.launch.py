import os
import subprocess
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

DB_PATH = '/mnt/ssd/sftp/rtabmapdb/rtabmap.db'

def check_db(context):
    """Verify DB integrity on startup. Delete if corrupted so rtabmap doesn't crash."""
    if not os.path.exists('/mnt/ssd'):
        print('[run.launch] WARNING: /mnt/ssd not mounted — rtabmap will use DB path as-is')
        return []

    if not os.path.exists(DB_PATH):
        print('[run.launch] No existing DB found — starting fresh')
        os.makedirs(os.path.dirname(DB_PATH), exist_ok=True)
        return []

    try:
        result = subprocess.run(
            ['sqlite3', DB_PATH, 'SELECT COUNT(*) FROM Node;'],
            capture_output=True, timeout=5
        )
        if result.returncode != 0:
            print(f'[run.launch] WARNING: DB corrupted (sqlite3 error) — deleting and starting fresh')
            os.remove(DB_PATH)
        else:
            count = result.stdout.decode().strip()
            print(f'[run.launch] DB OK — {count} nodes, continuing from existing map')
    except subprocess.TimeoutExpired:
        print('[run.launch] WARNING: DB check timed out — deleting and starting fresh')
        os.remove(DB_PATH)
    except Exception as e:
        print(f'[run.launch] WARNING: DB check failed ({e}) — deleting and starting fresh')
        os.remove(DB_PATH)

    return []


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)

    # ── Livox Mid360 (lidar) ──────────────────────────────────────────────────
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
            'topic_1':      '/livox/lidar_192_168_2_40',
            'topic_2':      '/livox/lidar_192_168_2_41',
            'output_topic': '/livox/lidar',
            'output_frame': 'livox_frame',
        }]
    )

    # ── VectorNav VN300 (IMU) ─────────────────────────────────────────────────
    vectornav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('vectornav_udp_bridge'),
            'launch', 'run.launch.py'
        )),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Only used when vectornav is NOT running (e.g. replay without IMU)
    core_stabilized_dummy = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='core_stabilized_dummy',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'Core', 'Core_stabilized']
    )

    # ── Robot State Publisher (Rove) ──────────────────────────────────────────
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rove_description'),
            'launch', 'launch.py'
        )),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # ── RTABMap (mapping) ─────────────────────────────────────────────────────
    # delete_db=false → rtabmap continues from existing DB on restart.
    # The check_db() function above verifies DB integrity first so rtabmap
    # never crashes on a corrupted file.
    # Set delete_db=true on the command line to force a fresh map:
    #   ros2 launch rove_color_mapping run.launch.py delete_db:=true
    rtabmap_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rove_color_mapping'),
            'launch', 'rtabmap.launch.py'
        )),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'delete_db':    LaunchConfiguration('delete_db', default='false'),
            'rtabmapviz':   LaunchConfiguration('rtabmapviz', default='false'),
        }.items()
    )

    # ── Gscam2 (cameras) ─────────────────────────────────────────────────────
    gscam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rove_color_mapping'),
            'launch', 'gscam.launch.py'
        ))
    )

    # ── Nav2 (costmaps) — disabled by default ────────────────────────────────
    # nav2_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(
    #         get_package_share_directory('rove_color_mapping'),
    #         'launch', 'nav.launch.py'
    #     )),
    #     launch_arguments={'use_sim_time': use_sim_time}.items()
    # )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulated clock (for bag replay).'),
        DeclareLaunchArgument(
            'delete_db', default_value='false',
            description='Delete existing rtabmap DB on start (forces fresh map).'),
        DeclareLaunchArgument(
            'rtabmapviz', default_value='false',
            description='Launch rtabmap_viz GUI (disable on headless Jetson).'),

        # DB integrity check — runs before rtabmap starts
        OpaqueFunction(function=check_db),

        livox_launch,
        lidar_merger,
        vectornav_launch,
        # core_stabilized_dummy,  # uncomment only if vectornav is NOT running
        robot_state_publisher,
        rtabmap_lidar_launch,
        gscam_launch,
        # nav2_launch,
    ])