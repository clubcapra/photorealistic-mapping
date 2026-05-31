# Rove Photo-realistic Mapping

This repository is more of a test (as of writing these lines) than anything, this is figure to change.


## Prerequisites: 
- Livox MID 360
- ROS 2 humble
- Vectornav VN-300

## How to build:
For first build:
```bash
cd src/livox_ros_driver2
./build.sh humble
cd ../../
source install/setup.bash
```
For subsequent builds:
```bash
colcon build --symlink-install --packages-ignore livox_ros_driver2 && source install/setup.bash
```
or use alias ``rosbuild`` if on the jetson

## How to run

```bash
ros2 launch rove_color_mapping run.launch.py
```

## Rosbag
To record a bag, use:
```bash
rosbag record -a -O my_recording.bag
```

To replay a bag to rebuild a rtabmap, use:
```bash
rosbag play my_recording.bag --topics \
  /livox/lidar \
  /livox/lidar/deskewed \
  /livox/imu \
  /imu/data \
  /imu/data_raw \
  /imu/mag \
  /imu/pressure \
  /imu/temperature \
  /scan \
  /input_scan \
  /input_scan/deskewed \
  /gps/fix \
  /fix \
  /tf \
  /tf_static \
  /joint_states \
  /robot_description
```




Test out rtsp 
gst-launch-1.0 \
  rtspsrc location=rtsp://192.168.2.33:554/ latency=0 protocols=tcp \
    drop-on-latency=true is-live=true \
  ! application/x-rtp,media=video,encoding-name=H265 \
  ! rtph265depay \
  ! h265parse config-interval=-1 \
  ! avdec_h265 \
  ! videoconvert \
  ! queue max-size-buffers=1 leaky=downstream \
  ! autovideosink sync=false