# Rove Photo-realistic Mapping

This repository is more of a test (as of writing these lines) than anything, this is figure to change.

## Prerequisites: 
- Livox MID 360
- ROS 2 humble
- Livox SDK 2

## Rosbag
To record a bag, use:
```bash
ros2 bag record -a -o my_recording
```

To replay a bag to rebuild a rtabmap, use:
```bash
ros2 bag play my_recording --topics \
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