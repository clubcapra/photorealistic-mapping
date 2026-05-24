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
from launch.actions import DeclareLaunchArgument
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
    # <topicName>/point_cloud — i.e. /livox/lidar/point_cloud. The tuner template
    # subscribes to /livox/lidar, so republish under that name.
    lidar_relay = Node(
        package='topic_tools',
        executable='relay',
        name='lidar_relay',
        arguments=['/livox/lidar/point_cloud', '/livox/lidar'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return [rove_driver, robot_state_publisher, lidar_relay]


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
