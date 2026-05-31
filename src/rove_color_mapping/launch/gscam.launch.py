import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

camera_name = 'front_camera'

config_dir = os.path.join(get_package_share_directory('rove_color_mapping'), 'config')
params_file = os.path.join(config_dir, 'gscam_test.yaml')

def generate_launch_description():

    gscam_node = Node(
        package='gscam2',
        executable='gscam_main',
        output='screen',
        name='gscam_publisher',
        namespace=camera_name,
        parameters=[
            params_file,
            {'camera_info_url': 'file://' + os.path.join(config_dir, 'front_camera.yaml')}
        ],
    )

    urdf_file = os.path.join(get_package_share_directory('rove_color_mapping'), 'urdf', 'sensor_mount.urdf.xacro')
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

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_link',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'world',
                   '--child-frame-id', 'base_link'],
    )

    rviz_config = os.path.join(config_dir, 'sensor_mount.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
    )

    return LaunchDescription([
        gscam_node,
        robot_state_publisher,
        joint_state_publisher,
        static_tf,
        rviz_node,
    ])