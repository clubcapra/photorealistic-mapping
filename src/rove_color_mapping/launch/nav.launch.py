from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('rove_color_mapping'),
        'config', 'nav2_costmap.yaml'
    )

    local_costmap = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='local_costmap',
        namespace='local_costmap',
        output='screen',
        parameters=[params],
    )

    global_costmap = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='global_costmap',
        namespace='global_costmap',
        output='screen',
        parameters=[params],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['local_costmap/local_costmap', 'global_costmap/global_costmap']
        }]
    )

    return LaunchDescription([
        local_costmap,
        global_costmap,
        lifecycle_manager,
    ])