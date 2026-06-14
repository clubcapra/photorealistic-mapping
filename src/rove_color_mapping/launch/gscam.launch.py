import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

config_dir = os.path.join(
    get_package_share_directory('rove_color_mapping'), 'config'
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

# Which cameras to rectify (publish <ns>/image_rect). Only cam_north feeds
# rtabmap live; the rest are rectified too so they can be eyeballed in rviz.
RECTIFY_CAMERAS = ['cam_north', 'cam_east', 'cam_south', 'cam_west']

# cv2.fisheye undistort balance: 0.0 crops to valid pixels, 1.0 keeps all
# source pixels (curved black borders). See note in fisheye_rectify.py.
RECTIFY_BALANCE = 0.0


def generate_launch_description():
    nodes = []
    for name, url in CAMERAS:
        # Each camera has its own (remapped) fisheye calibration.
        camera_info_path = os.path.join(config_dir, f'camera_info_{name}.yaml')
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

    # Fisheye -> pinhole rectification (rtabmap needs rectified RGB).
    nodes.append(Node(
        package='rove_color_mapping',
        executable='fisheye_rectify',
        name='fisheye_rectify',
        output='screen',
        parameters=[{
            'cameras':   RECTIFY_CAMERAS,
            'balance':   RECTIFY_BALANCE,
            'fov_scale': 1.0,
            'image_qos': 'sensor_data',
        }],
    ))
    return LaunchDescription(nodes)