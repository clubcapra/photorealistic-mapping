import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

config_dir = os.path.join(get_package_share_directory('rove_color_mapping'), 'config')
params_file = os.path.join(config_dir, 'gscam_test.yaml') #TODO change this to file gscam_jetson when on JETSON
camera_info_path = os.path.join(
    get_package_share_directory('rove_color_mapping'),
    'config', 'camera_info.yaml'
)


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
        ('cam_north', 'rtsp://192.168.2.35:554/'),  # was cam_east
        ('cam_east',  'rtsp://192.168.2.32:554/'),  # was cam_south  
        ('cam_south', 'rtsp://192.168.2.34:554/'),  # was cam_west
        ('cam_west',  'rtsp://192.168.2.33:554/'),  # was cam_north
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
                'camera_info_url': f'file://{camera_info_path}',
                'camera_name':     name,
                'frame_id':        f'{name}_optical_frame',
                'sync_sink':       False,
                'image_encoding':  'rgb8',
            }],
        ))

    return LaunchDescription(
        nodes
    )