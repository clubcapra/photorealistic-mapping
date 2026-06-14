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

## Run on boot (systemd)

On the robot (Jetson), [`scripts/update_scripts.sh`](scripts/update_scripts.sh)
installs two **user-level** systemd services so the mapping stack comes up
automatically on boot:

| Service                        | What it runs                                   |
|--------------------------------|------------------------------------------------|
| `rove_mapping_launch.service`  | `ros2 launch rove_color_mapping run.launch.py` |
| `rove_mapping_api.service`     | `python3 mapping_api.py`                        |

Both run under `ROS_DOMAIN_ID=96` and log to `/mnt/ssd/sftp/log/ros2`.

Prerequisites: the workspace must already be cloned and built at
`/home/capra/projects/photorealistic-mapping` (the unit files hardcode this
path), and `/mnt/ssd` must be mounted.

Deploy (run as `capra`, **not** root — it uses `sudo` where needed):

```bash
./scripts/update_scripts.sh
```

The script enables user lingering (`loginctl enable-linger capra`) so the
services start at boot and survive logout, then enables and (re)starts both.

Manage / inspect:

```bash
systemctl --user status rove_mapping_launch.service
systemctl --user status rove_mapping_api.service
journalctl --user -u rove_mapping_launch.service -f
```

## Rosbag
To record a bag, use:
```bash
ros2 bag record -a -o my_recording
```

To replay a bag to rebuild a rtabmap map, use:
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
```

And then in a seperate terminal run:
```bash
ros2 launch rove_color_mapping run.launch.py use_sim_time:=True
```
