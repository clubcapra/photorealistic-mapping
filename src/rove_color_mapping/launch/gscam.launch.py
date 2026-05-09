import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

camera_name = 'front_camera'

# Point to YOUR package's config
config_dir = os.path.join(get_package_share_directory('rove_color_mapping'), 'config')
params_file = os.path.join(config_dir, 'gscam.yaml')

def generate_launch_description():

    gscam_node = Node(
        package='gscam2',
        executable='gscam_main',   # ← correct executable name
        output='screen',
        name='gscam_publisher',
        namespace=camera_name,
        parameters=[params_file],
        remappings=[
            ('/image_raw', '/' + camera_name + '/image_raw'),
            ('/camera_info', '/' + camera_name + '/camera_info'),
        ],
    )

    return LaunchDescription([gscam_node])