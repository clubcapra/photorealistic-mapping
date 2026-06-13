from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default=False)

    params = os.path.join(
        get_package_share_directory('rove_color_mapping'),
        'config', 'nav2_costmap.yaml'
    )

    local_costmap = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        output='screen',
        parameters=[params],
    )
    

    # global_costmap = Node(
    #     package='nav2_costmap_2d',
    #     executable='nav2_costmap_2d',
    #     output='screen',
    #     parameters=[params],
    #     remappings=[('costmap/costmap', 'global_costmap/costmap')],
    # )
    

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['costmap/costmap']
        }]
    )

    return LaunchDescription([
        local_costmap,
        # global_costmap,
        lifecycle_manager,
    ])