import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

camera_info_path = os.path.join(
    get_package_share_directory('rove_color_mapping'),
    'config', 'camera_info.yaml'
)

# GStreamer pipeline template
PIPELINE = (
    'rtspsrc location={url} latency=0 protocols=tcp '
    'drop-on-latency=true is-live=true '
    '! application/x-rtp,media=video,encoding-name=H265 '
    '! rtph265depay ! h265parse config-interval=-1 '
    '! avdec_h265 ! videoconvert '
    '! queue max-size-buffers=1 leaky=downstream'
)

# (ros_name, rtsp_url)
CAMERAS = [
    ('cam_north', 'rtsp://192.168.2.35:554/'),
    ('cam_east',  'rtsp://192.168.2.32:554/'),
    ('cam_south', 'rtsp://192.168.2.34:554/'),
    ('cam_west',  'rtsp://192.168.2.33:554/'),
]


def generate_launch_description():
    nodes = []
    for name, url in CAMERAS:
        nodes.append(Node(
            package='gscam2',
            executable='gscam_main',
            name=f'{name}_gscam',        # unique name per camera
            namespace=name,
            output='screen',
            parameters=[{
                'gscam_config':    PIPELINE.format(url=url),
                'camera_info_url': f'file://{camera_info_path}',
                'camera_name':     name,
                'frame_id':        f'{name}_optical_frame',
                'sync_sink':       False,
                'preroll':         False,
                'use_gst_timestamps': False,
                'image_encoding':  'rgb8',
            }],
        ))
    return LaunchDescription(nodes)