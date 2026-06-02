import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

camera_name = 'front_camera'

config_dir = os.path.join(get_package_share_directory('rove_color_mapping'), 'config')
params_file = os.path.join(config_dir, 'gscam_test.yaml')

def generate_launch_description():
    
    gscam_pipeline = (
        'rtspsrc location={url} latency=0 protocols=tcp '
        'drop-on-latency=true is-live=true '
        '! application/x-rtp,media=video,encoding-name=H265 '
        '! rtph265depay ! h265parse config-interval=-1 '
        '! avdec_h265 ! videoconvert '
        '! queue max-size-buffers=1 leaky=downstream'
    )

    nodes = []

    camera_configs = [
    ('cam_north',  'rtsp://192.168.2.32:554/'),
    ('cam_south',   'rtsp://192.168.2.33:554/'),
    ('cam_east',  'rtsp://192.168.2.34:554/'),
    ('cam_west',   'rtsp://192.168.2.35:554/'),
    ]
    
    for name, url in camera_configs:
        nodes.append(Node(
            package='gscam2',
            executable='gscam_main',
            name='gscam_publisher',
            namespace=name,
            output='screen',
            parameters=[{
                'gscam_config':    gscam_pipeline.format(url=url),
                'camera_info_url': f'file://rove_color_mapping/config/camera_info.yaml',
                'camera_name':     name,
                'frame_id':        f'{name}_optical_frame',
                'sync_sink':       False,
                'image_encoding':  'rgb8',
            }],
        ))
    
    # urdf_file = os.path.join(get_package_share_directory('rove_color_mapping'), 'urdf', 'sensor_mount.urdf.xacro')
    # robot_description = xacro.process_file(urdf_file).toxml()

    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='screen',
    #     parameters=[{
    #         'robot_description': robot_description,
    #         'publish_frequency': 50.0,
    #     }]
    # )

    # joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    # )

    # static_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='world_to_base_link',
    #     arguments=['--x', '0', '--y', '0', '--z', '0',
    #                '--roll', '0', '--pitch', '0', '--yaw', '0',
    #                '--frame-id', 'world',
    #                '--child-frame-id', 'base_link'],
    # )

    # rviz_config = os.path.join(config_dir, 'sensor_mount.rviz')
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
    # )

    return LaunchDescription(
        nodes
    )