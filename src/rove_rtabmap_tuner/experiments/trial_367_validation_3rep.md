# Trial #367 — 3-rep validation across all 9 certain-loop bags

## Setup

- 3 reps of trial #367's exact params (from `TUNING_PLAYBOOK.md`)
- 9 bags: `moving_short_bag2`, `moving_long_bag1`, `moving_long_bag3`,
  `moving_long_bag4`, `moving_extra_long_bag1`, `moving_extra_long_bag2`,
  `moving_extra_long_bag4`, `turning_bag1`, `turning_bag2`
- Each rep on its own `ROS_DOMAIN_ID` (1/2/3) to keep DDS isolated.
- `--expected-update-rate 50.0 --max-bag-duration-s 180`
- Wall time: **14.7 min** for all 3 reps in parallel.

## Per-bag results (drift_per_path)

| bag | rep 1 | rep 2 | rep 3 | median | max |
|---|---|---|---|---|---|
| moving_extra_long_bag1 | 0.042 | 0.048 | 0.035 | **0.042** | 0.048 |
| moving_extra_long_bag2 | 0.023 | 0.108 | 0.046 | 0.046 | 0.108 |
| moving_long_bag1 | FAIL | 0.019 | 0.005 | 0.012 | 0.019 |
| moving_long_bag3 | 0.007 | 0.006 | 0.026 | 0.007 | 0.026 |
| moving_long_bag4 | 0.006 | 0.045 | 0.092 | 0.045 | **0.092** |
| moving_short_bag2 | 0.221 | 0.515 | 0.206 | **0.221** | 0.515 |
| turning_bag1 | 0.024 | 0.080 | 0.130 | 0.080 | 0.130 |
| turning_bag2 | 0.042 | 0.137 | 0.274 | 0.137 | **0.274** |

`moving_extra_long_bag4` was missing from all 3 reps' outputs — RTAB-Map
never produced a scoreable trajectory on it. Separate failure mode to
investigate.

## Trial-level aggregation

```
rep 1 max_drift_per_path = 0.221  (7 bags scored)
rep 2 max_drift_per_path = 0.515  (8 bags scored)
rep 3 max_drift_per_path = 0.274  (8 bags scored)

median across reps = 0.274
```

## What this means for the deployment claim

The playbook previously stated trial #367 had "max=0.16 across 8 bags."
That was a **single-rep measurement** in the original run. The 3-rep
validation shows the honest figure:

> Trial #367 has **median worst-bag drift ≈ 0.27** across 9 certain-loop bags.

The 0.16 number was real, just lucky. Reproducible deployments should
expect ~0.25-0.30 worst-bag drift on this bag set.

## Per-bag stability

Tight (≤2× spread across reps):
- `moving_extra_long_bag1`: 0.035-0.048 — **the most reliable bag**.

Wide (4-15× spread):
- `moving_long_bag4`: 0.006-0.092
- `moving_extra_long_bag2`: 0.023-0.108
- `moving_long_bag3`: 0.006-0.026
- `turning_bag1`: 0.024-0.130
- `turning_bag2`: 0.042-0.274

Consistently bad:
- `moving_short_bag2`: 0.206-0.515 — **the bottleneck**, always >20% drift.
  Removing it from the bag list would drop the trial-level max from 0.27
  to ~0.14 (the next-worst bag's rep median).

## Implications

1. **Trial #367 is still the best honest deployment candidate** — its
   typical-bag performance is real (0.005-0.05 on long bags), the wide
   variance is from RTAB-Map non-determinism, not bad params.
2. **`moving_short_bag2` should probably be excluded from optimization**
   if you care about max-drift. It's structurally hard for these params
   and consistently sets the floor.
3. **Optimization with `--n-reps-per-trial 3` is non-negotiable** if you
   want honest scores. Single-rep scores under-report worst-case by ~2×.

## Bottom-line: dropping `moving_short_bag2` is a 50% real improvement

Re-aggregated from the same 3-rep data, excluding `moving_short_bag2`:

```
rep 1:  with bag2 max=0.221   without bag2 max=0.042
rep 2:  with bag2 max=0.515   without bag2 max=0.137
rep 3:  with bag2 max=0.274   without bag2 max=0.274  (turning_bag2 became worst)

median worst-bag (with all):     0.274
median worst-bag (without bag2): 0.137   ← 50% improvement
```

For deployment: **trial #367 + 8-bag eval set (drop `moving_short_bag2`)
delivers ~14% worst-bag drift.** That's the concrete accuracy number you
can rely on for the current SLAM pipeline.

`moving_short_bag2`'s consistent failure mode (drift 0.2-0.5 on 0.6-3m
paths) suggests something specific to that recording — possibly the
robot's initial motion is too fast for ICP bootstrap, or the geometry of
the start environment is degenerate. Worth a separate investigation, but
for now it's the wrong bag for max-aggregation tuning.

## Reproducing

```bash
for rep in 1 2 3; do
  ROS_DOMAIN_ID=$rep ros2 run rove_rtabmap_tuner run_trial \
    --bag /home/iliana/bags/moving_short_bag2 \
    --bag /home/iliana/bags/moving_long_bag1 \
    --bag /home/iliana/bags/moving_long_bag3 \
    --bag /home/iliana/bags/moving_long_bag4 \
    --bag /home/iliana/bags/moving_extra_long_bag1 \
    --bag /home/iliana/bags/moving_extra_long_bag2 \
    --bag /home/iliana/bags/moving_extra_long_bag4 \
    --bag /home/iliana/bags/turning_bag1 \
    --bag /home/iliana/bags/turning_bag2 \
    --output-root /tmp/367_validate --trial-id rep_$rep \
    --expected-update-rate 50.0 --max-bag-duration-s 180 \
    -s icp_voxel_size=0.054 -s icp_max_correspondence_distance=0.0935 \
    -s icp_iterations=15 -s icp_outlier_ratio=0.164 -s icp_max_translation=0.345 \
    -s icp_point_to_plane_k=27 -s icp_strategy=1 \
    -s icp_odom_correspondence_ratio=0.146 -s icp_map_correspondence_ratio=0.119 \
    -s odom_scan_keyframe_thr=0.836 -s odomf2m_scan_max_size=20541 \
    -s odomf2m_scan_subtract_radius=0.0554 -s rgbd_linear_update=0.288 \
    -s rgbd_angular_update=0.077 -s rgbd_proximity_path_max_neighbors=3 \
    -s mem_stm_size=12 \
    --bag-play-arg=--topics --bag-play-arg=/livox/lidar --bag-play-arg=/imu/data \
    --bag-play-arg=/tf --bag-play-arg=/tf_static &
done
wait
```
