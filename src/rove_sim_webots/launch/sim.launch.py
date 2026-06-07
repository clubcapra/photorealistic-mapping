"""Launch Webots + the Rove ROS 2 driver.

Usage:
    ros2 launch rove_sim_webots sim.launch.py world:=outdoor_terrain.wbt mode:=realtime

The structure (Webots + supervisor + driver + reset/shutdown handlers) follows
the canonical webots_ros2 launch pattern; see e.g. webots_ros2_husarion.
"""

import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher


def _get_ros2_nodes(port: str = '1234'):
    """`port` must match the WebotsLauncher's port so the controller connects."""
    pkg_share = get_package_share_directory('rove_sim_webots')
    urdf_path = os.path.join(pkg_share, 'urdf', 'rove_webots.urdf')

    rove_driver = WebotsController(
        robot_name='rove',
        port=port,
        parameters=[
            {'robot_description': urdf_path},
            {'use_sim_time': True},
        ],
    )

    # robot_state_publisher with an empty description so it joins the TF tree
    # without conflicting with the static TFs the RoveDriver publishes.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>',
            'use_sim_time': True,
        }],
    )

    # webots_ros2_driver publishes the 3D Lidar PointCloud2 on
    # <topicName>/point_cloud. We have two lidars (matching the real Rove's
    # dual-MID-360 mount). Three relays:
    #   top lidar  → /livox/lidar_192_168_2_40
    #   bottom     → /livox/lidar_192_168_2_41
    #   /livox/lidar alias (= top) for SLAM defaults that hardcode that name.
    lidar_top_relay = Node(
        package='topic_tools', executable='relay', name='lidar_top_relay',
        arguments=['/livox/lidar_192_168_2_40/point_cloud',
                    '/livox/lidar_192_168_2_40'],
        parameters=[{'use_sim_time': True}], output='screen',
    )
    lidar_bot_relay = Node(
        package='topic_tools', executable='relay', name='lidar_bot_relay',
        arguments=['/livox/lidar_192_168_2_41/point_cloud',
                    '/livox/lidar_192_168_2_41'],
        parameters=[{'use_sim_time': True}], output='screen',
    )
    # Merger: combine the two MID-360 clouds (in their own frames) into a
    # single /livox/lidar PointCloud2 (frame=livox_frame). Closer to the real
    # Rove's dual-lidar fusion than the previous "top-only alias" relay.
    # Run as a plain python3 script - not a console_scripts entry, so no
    # rebuild is needed when editing the merger logic.
    merger_script = os.path.normpath(os.path.join(
        pkg_share, '..', '..', '..', '..',
        'src', 'rove_sim_webots', 'scripts', 'livox_merger.py'))
    lidar_merger = ExecuteProcess(
        cmd=['python3', merger_script, '--ros-args', '-p', 'use_sim_time:=true'],
        output='screen', name='livox_merger',
    )
    # SLAM node subscribes to /imu/data; sim publishes /livox/imu_192_168_2_40
    # (matches real-bag topic). Relay so the IMU stream reaches SLAM.
    # rove_slam_node still needs `enable_imu_factor=true` to *use* it, which
    # isn't yet exposed as a ROS param - but this gets the data flowing.
    imu_relay = Node(
        package='topic_tools', executable='relay', name='imu_relay',
        arguments=['/livox/imu_192_168_2_40', '/imu/data'],
        parameters=[{'use_sim_time': True}], output='screen',
    )

    return [rove_driver, robot_state_publisher,
            lidar_top_relay, lidar_bot_relay, lidar_merger, imu_relay]


def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        'world', default_value='outdoor_terrain.wbt',
        description='World file name under share/rove_sim_webots/worlds/.',
    )
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='realtime',
        description='Webots run mode: realtime | fast | pause',
    )

    # Headless control: set env WEBOTS_GUI=false (or pass headless:=true) to disable
    # 3D rendering. This is required on servers without a GPU/X — software OpenGL
    # under Xvfb is ~25x slower than realtime, but --no-rendering brings it back.
    # WebotsLauncher.gui is evaluated at construction time, so we read it from env
    # rather than a (lazy) LaunchConfiguration.
    gui_enabled = os.environ.get('WEBOTS_GUI', 'true').strip().lower() not in (
        'false', '0', 'no', 'off',
    )

    # Webots port: needs to be unique across concurrent sims on the same host.
    # WebotsLauncher constructs WebotsController bound to this port — both must
    # agree. We read from env (WEBOTS_PORT) since WebotsLauncher takes a string
    # at construction time, not a LaunchConfiguration.
    port = str(os.environ.get('WEBOTS_PORT', '1234'))

    webots = WebotsLauncher(
        world=PathJoinSubstitution([
            FindPackageShare('rove_sim_webots'), 'worlds', LaunchConfiguration('world'),
        ]),
        mode=LaunchConfiguration('mode'),
        ros2_supervisor=True,
        gui=gui_enabled,
        port=port,
    )

    # Respawn driver nodes when the user resets the world via Webots' GUI.
    reset_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots._supervisor,
            on_exit=lambda *args: _get_ros2_nodes(port=port),
        )
    )

    # Shut everything down when Webots itself exits.
    shutdown_on_webots_exit = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots,
            on_exit=[
                launch.actions.UnregisterEventHandler(
                    event_handler=reset_handler.event_handler,
                ),
                launch.actions.EmitEvent(event=launch.events.Shutdown()),
            ],
        )
    )

    return LaunchDescription([
        world_arg,
        mode_arg,
        webots,
        webots._supervisor,
        shutdown_on_webots_exit,
        reset_handler,
        *_get_ros2_nodes(port=port),
    ])
