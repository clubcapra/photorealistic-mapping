# rove_standard.urdf

Flat URDF extracted from `/robot_description` published by the live Rove
robot on **2026-06-01**, during the camera-lidar bag capture
(`rosbag2_test_camera_lidars/rosbag2_2026_06_01-22_12_52_0.db3`). The
original download was truncated; the URDF was recovered by walking the
SQLite overflow page chain in the partial `.db3` (pages 265 → 10 → 11 →
12 → 13 → 14).

`rove_standard.metadata.yaml` is the (intact) rosbag2 metadata for the
same recording, kept here for reference.

## What it describes

- **Robot name:** `rove_standard`
- **54 links, 53 joints**, all fixed except `joint_revolute_21` (revolute,
  on `chinese_camera_4_pivot` → unclear if intentional).
- **2 lidars** — both DJI MID-360 (Livox internals), mounted on the
  shared `pole`:
  - `dji_mid360` at +0.587 m on the pole, `rpy=[1.5708, 0, 1.5708]`
  - `dji_mid360_2` at +0.574 m on the pole, `rpy=[-1.5708, 0, 1.5708]`
    (rolled 180° about X — likely an inverted mount).
- **4 cameras** (N/S/E/W) on the same pole, mapped through
  `chinese_camera*` link names to `cam_{north,south,east,west}_optical_frame`.
- **VN-300 IMU** as `vn300_vectornav`, fixed to `Core`.

The new mounting orientation differs from `livox.urdf.xacro` — the
hard-coded SLAM default lidar→base extrinsic in
[`rove_slam_node.cpp:50`](../../rove_slam_ros/src/rove_slam_node.cpp#L50)
(`xyz=(-0.30, 0.00, 0.318)  rpy=(0, 30°, 180°)`) does **NOT** match this
URDF. Before running SLAM against bags from this hardware revision, the
extrinsic must be recomputed from the URDF (compose the chain
`base_link ← Core ← pole_pivot ← pole ← dji_mid360_pivot ← dji_mid360`).

## Missing GLB meshes

The URDF references `package://rove_description/meshes/*.glb` for every
visual/collision (ASection, Base, BSection, drums, flippers, joints,
pole, chinese_camera*). The current `src/rove_description/meshes/` only
holds the older `.stl` files (`base.stl`, `flipper.stl`, `track.stl`,
`vlp16/...`). The `.glb` files would need to be pulled from the
live-system filesystem (or from whoever published the new URDF) before
this URDF is usable in rviz / Gazebo. **SLAM does not need the meshes**
— only the joint origins.

## Source bag

The bag the URDF came from had:

| Topic                           | Type                          | Msgs |
|---------------------------------|-------------------------------|------|
| `/cam_{north,south,east,west}/image_raw` | `sensor_msgs/Image`  | varies |
| `/cam_north/camera_info`        | `sensor_msgs/CameraInfo`      | 529  |
| `/livox/imu_192_168_2_40`       | `sensor_msgs/Imu`             | 5502 |
| `/livox/imu_192_168_2_41`       | `sensor_msgs/Imu`             | 5500 |
| `/livox/lidar_192_168_2_40`     | `sensor_msgs/PointCloud2`     | —    |
| `/livox/lidar_192_168_2_41`     | `sensor_msgs/PointCloud2`     | —    |
| `/robot_description`            | `std_msgs/String`             | 1    |
| `/tf`, `/tf_static`             | `tf2_msgs/TFMessage`          | —    |

The `192_168_2_40` and `192_168_2_41` suffixes are the lidars' IPs —
matching the `dji_mid360` and `dji_mid360_2` URDF links respectively.
