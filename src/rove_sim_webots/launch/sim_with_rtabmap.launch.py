"""Launch the Webots sim AND the tuner's rtabmap pipeline against it.

Pipeline:
    Webots (Rove) -> /livox/lidar + /livox/imu -> rtabmap (via tuner template)
    -> ~/.ros/<db_name>

Usage:
    ros2 launch rove_sim_webots sim_with_rtabmap.launch.py \\
        world:=outdoor_terrain.wbt db_name:=sim_outdoor_loop1.db
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_share = get_package_share_directory('rove_sim_webots')

    db_arg = DeclareLaunchArgument(
        'db_name', default_value='sim.db',
        description='Database filename written into ~/.ros/',
    )
    world_arg = DeclareLaunchArgument(
        'world', default_value='outdoor_terrain.wbt',
    )

    # The tuner ships a tunable rtabmap launch template at:
    #   src/rove_rtabmap_tuner/rove_rtabmap_tuner/templates/lidar3d_tunable.launch.py.tmpl
    # That's a Jinja template, not a directly-launchable file. For this sim
    # integration we use the rtabmap_launch package's stock launcher with the
    # same topic remappings the template uses.
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rtabmap_launch'), 'launch', 'rtabmap.launch.py',
            ]),
        ),
        launch_arguments={
            'frame_id': 'livox_frame',
            # rtabmap_launch arg name is scan_cloud_topic for the PointCloud2 input,
            # not lidar_topic (that's the tuner's template var, which maps to this).
            'scan_cloud_topic': '/livox/lidar',
            'imu_topic': '/livox/imu',
            'subscribe_scan_cloud': 'true',
            'subscribe_rgbd': 'false',
            'subscribe_depth': 'false',
            'subscribe_rgb': 'false',
            'icp_odometry': 'true',
            'use_sim_time': 'true',
            'database_path': ['~/.ros/', LaunchConfiguration('db_name')],
            # Disable RViz and rtabmap_viz; both crash with SIGABRT under
            # xvfb-run (no GL context for Qt), and we don't need them for
            # autonomous validation runs.
            'rviz': 'false',
            'rtabmap_viz': 'false',
            # Default Icp/MaxTranslation is 0.2 m — too tight when the sim
            # runs slower than realtime under load (scan gap > 0.2 m). Bump
            # it; also disable deskewing since we don't publish a fixed_frame.
            # Tuner orchestrator overrides via SIM_EXTRA_RTABMAP_ARGS env var
            # so each trial can swap in its own param set without rebuilding.
            'args': (
                os.environ.get('SIM_EXTRA_RTABMAP_ARGS',
                               '--Icp/MaxTranslation 2.0 --Icp/MaxRotation 1.5')
            ),
            'deskewing': 'false',
        }.items(),
    )

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, 'launch', 'sim.launch.py'),
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items(),
    )

    return LaunchDescription([
        world_arg,
        db_arg,
        sim_launch,
        # Delay rtabmap a couple of seconds so Webots/TF are up first.
        TimerAction(period=3.0, actions=[rtabmap_launch]),
    ])
