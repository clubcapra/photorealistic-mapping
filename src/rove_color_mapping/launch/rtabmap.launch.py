import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default=False)

    # ── Use the official RTABMap lidar3d example launch ──────────────────────
    # Pass our specific topics and frame into it directly
    rtabmap_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rove_color_mapping'),
            'launch',
            'lidar3d.launch.py'
        )),
        launch_arguments={
            'frame_id':            'Core',
            'lidar_topic':         '/livox/lidar',
            'imu_topic':           '/imu/data',
            'deskewing':           'true',
            'voxel_size':          '0.1',
            'qos':                 '1',
            'expected_update_rate':'15.0',
            'use_sim_time':         use_sim_time,
        }.items()
        )
    return LaunchDescription(
        [
            rtabmap_lidar_launch
        ]
    )