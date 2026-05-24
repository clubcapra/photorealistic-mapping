# rove_sim_webots

Webots-based simulation of the Rove rover, designed to produce RTAB-Map databases
on demand. The simulated robot publishes the same topics the real bags do
(`/livox/lidar` on frame `livox_frame`, `/livox/imu`), so anything downstream
that works on a real bag — including `rove_rtabmap_tuner` — works on a sim run
with zero code changes.

## Why Webots over Gazebo

- Single `.wbt` text file per world; no SDF + xacro + plugin XML fan-out.
- ROS 2 integration is one URDF + one launch file (`webots_ros2_driver`).
- Runs on any GPU (we explicitly do not need photorealism here — the SLAM
  pipeline cares about geometry + IMU dynamics, not visual fidelity).
- Deterministic seeded terrain via `UnevenTerrain { randomSeed N }`.

## Layout

```
src/rove_sim_webots/
├── protos/Rove.proto              # 4-wheel skid-steer rover, sensors at the
│                                  # real-rig poses, body matches Rove footprint
├── worlds/
│   ├── outdoor_terrain.wbt        # Perlin-noise hills, palms/barrels for features
│   ├── indoor_structured.wbt      # 20m arena, interior walls, pallets/boxes
│   └── mixed.wbt                  # Outdoor lot + a building you can drive into
├── urdf/rove_webots.urdf          # webots_ros2_driver device->topic mappings
├── rove_sim_webots/
│   ├── rove_driver.py             # Plugin: cmd_vel -> wheels, /livox/imu, /odom, tf
│   ├── trajectories.py            # YAML segment loader
│   └── scripted_runner.py         # CLI: sim -> bag/db end-to-end
├── launch/
│   ├── sim.launch.py              # sim only
│   └── sim_with_rtabmap.launch.py # sim + rtabmap_launch -> ~/.ros/<name>.db
└── config/trajectories/*.yaml     # Reusable cmd_vel programs
```

## Install Webots (one-time)

```bash
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://cyberbotics.com/Cyberbotics.asc \
  | sudo gpg --dearmor -o /etc/apt/keyrings/cyberbotics.gpg
echo 'deb [arch=amd64 signed-by=/etc/apt/keyrings/cyberbotics.gpg] https://cyberbotics.com/debian binary-amd64/' \
  | sudo tee /etc/apt/sources.list.d/cyberbotics.list
sudo apt update && sudo apt install -y webots
sudo apt install -y ros-humble-webots-ros2 ros-humble-rtabmap-launch
```

Webots installs to `/usr/local/webots`. The ROS 2 driver picks it up via
`$WEBOTS_HOME` (set by `webots_ros2_driver` automatically when installed via apt).

## Build

```bash
cd ~/prog/photorealistic-mapping/.claude/worktrees/sim-webots
colcon build --packages-select rove_sim_webots
source install/setup.zsh    # zsh, not bash — see memory project-workspace-shell
```

## Quick start

All commands assume `ROS_DOMAIN_ID` is in 120-140 (see memory feedback-ros-domain-range);
the scripted runner sets it itself.

End-to-end: drive the outdoor loop trajectory, get an RTAB-Map db out:

```bash
python -m rove_sim_webots.scripted_runner \
  --mode live \
  --world outdoor_terrain.wbt \
  --trajectory outdoor_loop1 \
  --out-dir ./sim_runs/outdoor_v1 \
  --db-name outdoor_v1.db
```

Record a bag for offline processing through the tuner:

```bash
python -m rove_sim_webots.scripted_runner \
  --mode record \
  --world indoor_structured.wbt \
  --trajectory indoor_corridor \
  --out-dir ./sim_runs/indoor_v1 \
  --bag-name indoor_v1
```

The bag is rosbag2 SQLite — drop it next to your other bags and feed it to
`rove_rtabmap_tuner` with the standard `lidar3d_tunable.launch.py.tmpl` template.

Just the sim, no automation:

```bash
ROS_DOMAIN_ID=130 ros2 launch rove_sim_webots sim.launch.py \
  world:=mixed.wbt
```

## Topic contract

| Topic                     | Type                        | Rate    | Frame         |
|---------------------------|-----------------------------|---------|---------------|
| `/livox/lidar`            | sensor_msgs/PointCloud2     | 10 Hz   | `livox_frame` |
| `/livox/imu`              | sensor_msgs/Imu             | ~200 Hz | `livox_frame` |
| `/rove/camera/image_raw`  | sensor_msgs/Image           | 15 Hz   | `camera_link` |
| `/rove/gps`               | sensor_msgs/NavSatFix       | 5 Hz    | `base_link`   |
| `/odom`                   | nav_msgs/Odometry           | step    | `odom`        |
| `/cmd_vel` (sub)          | geometry_msgs/Twist         | -       | -             |
| `/tf`, `/tf_static`       | tf2_msgs/TFMessage          | step    | -             |

This matches the defaults in `rove_rtabmap_tuner`'s
`templates/lidar3d_tunable.launch.py.tmpl`, so the tuner will subscribe with
no remapping.

## Add a new world

1. Drop a new `worlds/<name>.wbt` referencing `EXTERNPROTO "../protos/Rove.proto"`
   and instantiating `Rove { name "rove" ... }`.
2. Set `coordinateSystem "ENU"` in `WorldInfo` (the driver TFs assume that).
3. Run with `--world <name>.wbt`.

The `protos/Rove.proto` is shared across all worlds — sensor changes go there,
not in each `.wbt`.

## Add a new trajectory

Drop a YAML file under `config/trajectories/<name>.yaml`:

```yaml
name: my_trajectory
description: "What this trajectory exercises"
segments:
  - { dt: 5.0, v: 0.5, w: 0.0 }
  - { dt: 1.5, v: 0.0, w: 1.0 }   # ~ 90 deg turn at 1 rad/s
  ...
```

The runner resolves `--trajectory my_trajectory` from
`share/rove_sim_webots/config/trajectories/`.

## Cross-verification: `--mode validate`

End-to-end SLAM validation against ground truth. The Webots supervisor knows
the robot's true pose at every step; the validator pairs each ground-truth
sample with a time-matched RTAB-Map estimate and reports standard SLAM
benchmark metrics.

```bash
python3 -m rove_sim_webots.scripted_runner \
  --mode validate \
  --world outdoor_terrain.wbt \
  --trajectory outdoor_loop1 \
  --out-dir ./sim_runs/run1 \
  --headless
```

Produces in `<out-dir>/`:
- `validate_bag/` — rosbag with `/livox/lidar`, `/livox/imu`,
  `/ground_truth/odom` (true pose), `/rtabmap/odom`, `/rtabmap/mapPath`,
  `/tf`, `/tf_static`, `/clock`, `/cmd_vel`.
- `validate.db` — RTAB-Map database.
- `validation.json` — the cross-verification report:

```json
{
  "n_gt_poses": 389,
  "n_est_poses": 96,
  "n_pairs": 96,
  "duration_s": 16.5,
  "trajectory_length_m": 0.95,
  "ate_rmse_m": 0.27,           // RMS position error after Umeyama alignment
  "ate_mean_m": 0.24,
  "ate_median_m": 0.27,
  "ate_max_m": 0.50,
  "final_drift_m": 0.27,        // final-pose error (m)
  "final_drift_yaw_rad": 0.04,
  "drift_ratio": 0.28,          // final_drift / trajectory_length
  "alignment_translation_m": [0.44, -0.04, 0.90],
  "alignment_rotation_matrix": [[...], [...], [...]]
}
```

The validator (`rove_sim_webots.validator`) can also be run standalone on any
bag that contains `/ground_truth/odom` and any of `/rtabmap/mapPath`,
`/rtabmap/local_path`, `/rtabmap/global_path`, `/rtabmap/odom`, or
`/icp_odom`:

```bash
python3 -m rove_sim_webots.validator --bag /path/to/bag
```

### How it works

1. `Rove.proto` sets `supervisor TRUE`. The `rove_driver` plugin calls
   `robot.getSelf().getPosition() / .getOrientation()` every Webots step and
   publishes `/ground_truth/odom` (nav_msgs/Odometry, frame `world`, child
   `base_link`, stamped with sim time).
2. `--mode validate` launches `sim_with_rtabmap.launch.py` (sim + ICP
   odometry + RTAB-Map), records the bag, drives the trajectory, and tears
   down cleanly.
3. After teardown, `validator.validate(bag)`:
   - Reads both trajectories.
   - Time-associates each estimated pose with the nearest ground-truth pose
     (default tolerance 50 ms).
   - Runs a closed-form Umeyama SE(3) alignment (no scale) on the matched
     pairs.
   - Reports ATE / mean / median / max position error, final-pose drift in
     position and yaw, and the SE(3) alignment.

### Known tuning gap (not an architecture gap)

The pipeline is end-to-end verified, but the **default RTAB-Map ICP settings
do not track this sensor mix well**:
- Webots `Lidar` produces a uniform rotating scan, not Livox's non-repeating
  pattern. Per-point timing distribution differs even though FoV/range
  match.
- The default `Icp/MaxTranslation=0.2 m` rejects inter-scan motion when the
  sim runs slower than realtime (heavy concurrent load → larger sim-time
  gaps between scans). `sim_with_rtabmap.launch.py` already bumps this to
  `2.0 m` and disables deskewing, but you'll likely need to tune further.
- IMU-initialised ICP guesses (set `wait_imu_to_init:=true` in
  `rtabmap_launch`) make a large difference once enabled.

On a smoke-test run with `--trajectory slow_short` (1 m at 0.1 m/s),
RTAB-Map produced a real odometry estimate (ATE=0.27 m, 28% drift_ratio) —
proof the pipeline works, but the SLAM result itself needs tuning. **Tune
`sim_with_rtabmap.launch.py` `args` with your tuner's best params** to get
useful validation numbers.

### Map-quality validation (`map_validator.py`)

Stub — see [map_validator.py](rove_sim_webots/map_validator.py) for the
planned chamfer-distance-vs-world-geometry interface. Implement when the
trajectory metric stops being the bottleneck.

## Headless / server runs

Webots needs an OpenGL context even with no GUI. For autonomous runs on a
server with no display:

```bash
sudo apt install xvfb
python -m rove_sim_webots.scripted_runner --mode record --headless ...
```

`--headless` does two things:
1. Sets `WEBOTS_GUI=false`, which passes `--no-rendering` to Webots (disables
   3D rendering — the simulator only does physics + sensor ray casting).
2. If no `$DISPLAY` is set, wraps the launch in `xvfb-run -a` so Webots' Qt
   bootstrap finds an X server. (If you're already inside an X session or
   another virtual display, `--headless` just sets the env var and does not
   add `xvfb-run` again.)

**Measured speeds** (47 s "outdoor_loop1" trajectory):

| Setup                              | /livox/lidar | /livox/imu | bag size |
|------------------------------------|--------------|------------|----------|
| GUI on a real X with iGPU          | 8.5 Hz       | 60 Hz      | 224 MiB  |
| `--headless` (`xvfb-run` + no-render) | 4.7 Hz    | 33 Hz      | 125 MiB  |
| `xvfb-run` without `--no-rendering` | 0.3 Hz       | 2.3 Hz     | 7.9 MiB  |

The middle row is real-time (47s sim took 47s wall) under heavy CPU contention
from a parallel tuner study. On an idle server it should approach GUI rates.
Skipping `--headless` (or `--no-rendering`) on a server forces software OpenGL
in Xvfb and is ~25× slower — never the right move.

For GPU-equipped servers with no display, EGL surfaceless contexts work but
require setting `__EGL_VENDOR_LIBRARY_FILENAMES` and similar; the `xvfb-run`
+ `--no-rendering` path above is simpler and good enough for SLAM data
generation since we don't need rendered output.

## Caveats / known gaps

- **Locomotion is 4-wheel skid-steer, not tracks.** Wheel-odometry behavior
  differs from the real Rove's track odometry, especially on slopes. For
  SLAM-data generation this is fine (RTAB-Map uses ICP odometry from the lidar
  by default in our template) but don't trust `/odom` as a stand-in for the
  real track odometer.
- **LiDAR is a uniform `Lidar` node, not Livox-pattern.** Webots doesn't model
  the non-repeating Livox scan pattern. The point cloud rate, FoV, and range
  match Livox-Mid-40 specs, but per-point timing distribution differs. Should
  not affect RTAB-Map's scan-matching, but be aware if you tune deskewing.
- **Track friction is approximated, not modeled.** Set `coulombFriction` per
  world if a specific run slips too much.
- **No multi-Rove yet.** One robot per world; `name "rove"` is hard-coded in
  the controller.

## Sanity check before committing changes

```bash
colcon build --packages-select rove_sim_webots
source install/setup.zsh
ros2 launch rove_sim_webots sim.launch.py world:=outdoor_terrain.wbt &
sleep 30
ros2 topic hz /livox/lidar    # expect ~10 Hz
ros2 topic hz /livox/imu      # expect ~100-200 Hz
kill %1
```
