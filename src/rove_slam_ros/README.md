# rove_slam_ros

ROS 2 wrapper around the [`rove_slam`](https://github.com/clubcapra/rove_slam)
SLAM core. Vendors the SLAM core as a git submodule under
[external/rove_slam](external/rove_slam) and exposes it as a single
`rove_slam_node` executable plus launch files for SLAM-only, SLAM + nav2
local-costmap, and rosbag2 headless replay.

## Quick start

```sh
# 1. Clone the workspace with submodules.
git clone --recursive <photorealistic-mapping>
cd photorealistic-mapping

# (or if you already cloned without --recursive)
git submodule update --init --recursive

# 2. Source ROS 2 Humble and build.
source /opt/ros/humble/setup.bash
colcon build --packages-up-to rove_slam_ros --symlink-install
source install/setup.bash

# 3. Run SLAM standalone on a live Livox + VN-300 setup.
ros2 launch rove_slam_ros slam.launch.py

# 4. Run SLAM + nav2 local costmap headless (drives /costmap from /cloud_obstacles).
ros2 launch rove_slam_ros slam_nav.launch.py

# 5. Replay a rosbag2 into the same SLAM + nav stack.
ros2 launch rove_slam_ros bag_replay.launch.py \
    bag:=/home/iliana/bags/moving_extra_long_bag2 \
    with_nav:=true
```

## Topics

Subscribes:
- `/livox/lidar` (`sensor_msgs/PointCloud2`) — lidar input. Accepts both Livox
  PointCloud2 and standard XYZ+intensity layouts; per-point time is honored
  when present.
- `/imu/data` (`sensor_msgs/Imu`) — VN-300 IMU. Buffered for Phase 3.1
  loop-closure work; currently has no effect on the lidar-only front end.

Publishes:
- `/tf` — `map_frame → odom_frame → base_frame` (configurable, see
  [config/slam.yaml](config/slam.yaml)).
- `/odom` (`nav_msgs/Odometry`) — body-in-world pose.
- `/cloud_obstacles` (`sensor_msgs/PointCloud2`) — Z-banded local map for
  nav2 obstacle layer. Republished every `obstacle_publish_period_s`
  (default 0.5 s).

## Parameters

All node parameters live in [config/slam.yaml](config/slam.yaml) with
inline comments. Override by passing your own file:

```sh
ros2 launch rove_slam_ros slam.launch.py params_file:=/path/to/my.yaml
```

| Group           | Param                       | Default        | Notes                                                  |
|-----------------|-----------------------------|---------------:|--------------------------------------------------------|
| Frames          | `map_frame`                 | `map`          | World frame published by SLAM                          |
|                 | `odom_frame`                | `odom`         | Intermediate odom frame                                |
|                 | `base_frame`                | `base_link`    | Robot body frame                                       |
| KISS-ICP        | `voxel_size_m`              | `0.30`         | Hash map voxel size; tuned for non-rep MID-360 indoor |
|                 | `max_range_m`               | `100.0`        | Far point cutoff                                       |
|                 | `min_range_m`               | `2.0`          | Near point cutoff (chassis self-returns)               |
|                 | `max_points_per_voxel`      | `50`           | Caps stored points per voxel                           |
|                 | `deskew`                    | `true`         | Per-point timestamp deskew                             |
|                 | `min_intensity`             | `0.0`          | Intensity floor                                        |
|                 | `urdf_extrinsic`            | `true`         | Apply URDF lidar→base transform                        |
| Obstacles       | `obstacle_z_min`            | `0.10`         | Floor of Z-band (m)                                    |
|                 | `obstacle_z_max`            | `1.50`         | Ceiling of Z-band (m)                                  |
|                 | `obstacle_publish_period_s` | `0.5`          | Republish cadence (s)                                  |

nav2 costmap parameters are in [config/nav2_costmap.yaml](config/nav2_costmap.yaml).
`obstacle_max_range` is the biggest one — keep it ≤ the lidar's reliable
range (Livox MID-360 ≈ 12 m indoors).

## Submodule

The SLAM core lives at [external/rove_slam](external/rove_slam) as a git
submodule. CMake adds it as a subdirectory and links the
`rove_slam_core` library into the ROS node. The submodule's own
`ROVE_SLAM_BUILD_ROS_BRIDGE` option is force-OFF here (this package
provides the ROS interface), and the submodule's `ROVE_SLAM_USE_GTSAM`
is OFF by default (loop closure off for the bridge — turn ON if you
want phase-3.3 LC in the live stack; the upstream README documents
the trade-offs).

To pull a newer SLAM core revision:

```sh
cd src/rove_slam_ros/external/rove_slam
git fetch origin
git checkout <ref>          # e.g. phase-4, main, a SHA…
cd -
git -C . add src/rove_slam_ros/external/rove_slam
git commit -m "rove_slam: bump submodule"
```

## Smoke test

```sh
# Build, source, replay a bag into SLAM + nav, verify the costmap topic prints.
colcon build --packages-up-to rove_slam_ros --symlink-install
source install/setup.bash
ros2 launch rove_slam_ros bag_replay.launch.py \
    bag:=/home/iliana/bags/moving_extra_long_bag2 \
    with_nav:=true &
sleep 10
ros2 topic hz /odom            # ~10 Hz expected
ros2 topic hz /cloud_obstacles # ~2 Hz expected
ros2 topic echo /costmap/costmap --once | head -10
```

## Known issues

- nav2's `lifecycle_manager_costmap` can hang at `Configuring` on some
  setups; `bond_timeout: 0.0` in the launch file works around that. Cause
  is environment-specific (observed without our bridge too); not us.
- VN-300 calibration is currently uncalibrated, so IMU plumbing is wired
  but disabled. See the upstream `docs/phase-3.1` for the calibration
  procedure (it's mostly a one-time field-day task).
