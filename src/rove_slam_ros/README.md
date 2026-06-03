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

# 6. Or use the one-stop bringup with knobs for everything:
ros2 launch rove_slam_ros bringup.launch.py
# Common variants:
ros2 launch rove_slam_ros bringup.launch.py viewer:=true        # + live rerun
ros2 launch rove_slam_ros bringup.launch.py rviz:=true          # + rviz2
ros2 launch rove_slam_ros bringup.launch.py bag:=~/bags/xxx     # bag replay
ros2 launch rove_slam_ros bringup.launch.py drive:=true \
    rove_host:=jetson.local                                      # drive the Rove
ros2 launch rove_slam_ros bringup.launch.py mesh_method:=tsdf   # bounded-max mesh
```

## Mesh building (mesh_builder node)

A `mesh_builder` node is always running in the bringup. It buffers SLAM
trajectory + lidar scans live and produces a mesh on demand via a ROS
service. The reconstruction backend is chosen by the `mesh_method`
ROS parameter (default `poisson`):

```sh
# Pick a method at launch time:
ros2 launch rove_slam_ros bringup.launch.py mesh_method:=bpa
ros2 launch rove_slam_ros bringup.launch.py mesh_method:=tsdf
ros2 launch rove_slam_ros bringup.launch.py mesh_method:=poisson

# Trigger a build (from any shell with the workspace sourced):
ros2 service call /mesh_builder/build_mesh std_srvs/srv/Trigger

# Output (default /tmp/rove_slam_mesh):
#   trajectory.tum            live-accumulated SLAM trajectory
#   scans.rec/                buffered lidar scans (live only)
#   mesh_<method>.dense.pcd   intermediate dense cloud (bpa/poisson)
#   mesh_<method>.ply         the mesh
#   build.log                 tool stdout/stderr from that build
```

If `build_mesh_on_shutdown:=true` (default), the node also fires a build
when SIGINT/SIGTERM hits — useful for bag-replay tests where you want
the mesh right after the bag finishes.

**Backend comparison** (measured on bag2: 1071 scans, 18 × 23 m room):

| method  | mean   | median | p95    | max    | wall-clock | visual |
|---------|-------:|-------:|-------:|-------:|-----------:|:------:|
| poisson | 2.8 cm | 2.1 cm | 6.6 cm | 3.8 m  | 34 s       | smooth |
| tsdf    | 6.0 cm | 3.5 cm | 19 cm  | 1.2 m  | 23 s       | smooth |
| bpa     | 2.5 cm | 1.5 cm | 6.5 cm | 4.0 m  | 33 s       | **noisy** |
| nvblox  | planned: ~1-2 cm at 5 cm voxel (CUDA native lidar ray-tracer)    |

**Note on BPA**: the accuracy metric looks best for BPA, but that's
misleading — BPA preserves every input point as a mesh vertex, so the
surface inherits all per-scan SLAM noise verbatim. Visually it's the
worst of the three. Poisson smooths over the noise (implicit-function
fit), TSDF averages it out (voxel weights). Pick BPA only if a
downstream tool needs exact-point preservation.

**Note on Poisson "ballooning"**: Poisson invents surfaces in
unobserved regions — its max-error of 3.8 m comes from bulges into
the void rather than per-vertex noise. If your scan coverage isn't
near-full, prefer TSDF (bounded by construction).

**TSDF post-processing** (on by default in `tsdf_mesh.py`):
- Connected-component filter drops floaters smaller than 200 triangles.
- 5-iter Taubin smoothing (λ=0.5, μ=-0.53) denoises without shrinkage.
Adds ~0.2 s, gets TSDF visual quality to ~Poisson-equivalent without
the bulges. Disable with `--smooth-iters 0 --min-cluster-tris 0`.

Pick by use case:
- Live + color, NVIDIA target:         `nvblox`   (recommended on Jetson — see below)
- Live + color, CPU only:              `tsdf`     (with the colorize hook)
- Smooth display mesh, post-process:   `poisson`  (no live constraint)
- Exact-point preservation:            `bpa`      (rare; visually noisy)

### nvblox (CUDA live TSDF, on NVIDIA targets)

`mesh_method:=nvblox` brings up `nvblox_node` (from `isaac_ros_nvblox`)
alongside SLAM. It integrates lidar + camera color into a TSDF voxel
grid live during the run; the `~/build_mesh` service call ends up
saving the already-built mesh via `/nvblox_node/save_ply` instead of
running a CPU reconstruction. The output is per-vertex colored
natively, so the `colorize_mesh:=true` hook is a no-op.

Setup (one-time, NVIDIA hardware only):
```sh
# Either install isaac_ros_nvblox from source (Humble/Iron):
#   https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox
# Or use the NVIDIA pre-built Docker image (Jetson):
#   nvidia/isaac-ros-dev-aarch64
```

Without `nvblox_ros` installed, `mesh_method:=nvblox` fails fast at
launch with a clear "package not found" error and the user can switch
to `mesh_method:=tsdf` for the CPU-equivalent path.

The same backend selection is available offline through the CLI in the
SLAM submodule:
```sh
external/rove_slam/tools/build_mesh.py --method tsdf \
    --rec <recording.rec> --traj <trajectory.tum> --out mesh.ply
```

## Live visualization

Two options, both controlled by args on `bringup.launch.py`.

### rerun (recommended)

```sh
# One-time install (rerun 0.21 is the last that works with the numpy 1.x
# pinned by ROS Humble's Python bindings):
pip install --user 'rerun-sdk==0.21.0' 'numpy<2'

# Then enable the viewer alongside any bringup:
ros2 launch rove_slam_ros bringup.launch.py viewer:=true
```

A rerun window pops up showing the live TF tree, /odom trajectory, and
/cloud_obstacles, all on a scrubable timeline. Pass `viewer_raw:=true` to
also stream /livox/lidar (heavy — subsampled to 30 k points per scan).

To attach a remote viewer instead (so the viewer runs on your laptop, the
node runs on the robot), bypass the launch and run:
```sh
ros2 run rove_slam_ros rerun_live.py --ros-args -p serve:=true
# then from the laptop:
rerun --connect rerun+http://<robot-ip>:9876/proxy
```

### rviz2

```sh
ros2 launch rove_slam_ros bringup.launch.py rviz:=true
```

Opens rviz2 with [`config/rove_slam.rviz`](config/rove_slam.rviz) pre-wired
to TF, /cloud_obstacles, /livox/lidar (disabled by default), /odom,
/global_costmap, /local_costmap, and /plan. The Nav2 Goal tool is on the
toolbar — click it, drop a goal in the map view, and observe /cmd_vel.
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
