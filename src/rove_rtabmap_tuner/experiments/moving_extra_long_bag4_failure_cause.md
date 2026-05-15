# `moving_extra_long_bag4` failure mode — root cause identified

## Symptom

In trial #367's 3-rep validation (and every other eval run), `moving_extra_long_bag4` produced no scoreable trajectory. The bag was silently dropped from per-bag stats; trial-level aggregation skipped it.

## Root cause

Two structural issues with the bag itself:

1. **Bag duration 232 s** exceeds `--max-bag-duration-s 180` (our standard cap for tuning runs). When the cap expires, `ros2 bag play` is SIGINT'd before the recording finishes.

2. **Missing `/tf` and `/tf_static` topics.** Checked metadata.yaml:
   - `moving_long_bag1` has both `/tf` and `/tf_static` topics.
   - `moving_extra_long_bag4` does **not** have either. Looks like the recording captured RTAB-Map outputs (`/rtabmap/republish_node_data`, `/odom_local_scan_map`, etc.) and raw lidar/IMU (`/livox/lidar`, `/livox/imu`, `/imu/data`) but not the TF tree.

Without `/tf_static`, RTAB-Map can't transform incoming point clouds from the sensor frame into `base_link`, so the trajectory never bootstraps.

Verified with single-bag `run_trial` on `moving_extra_long_bag4` with `#367` params (ROS_DOMAIN_ID 85, log at `/tmp/bag4_diagnose/`):
```
[trial b4] bag moving_extra_long_bag4: FAIL (189.5s) —
  bag playback exceeded --max-bag-duration-s (180.0s)
```
Even bumping the cap to 240 would only fix issue (1). Issue (2) is fatal for SLAM evaluation.

## Disposition

- **Permanent exclusion from the eval set** is correct. This bag is a structural casualty of its recording, not a tuning challenge.
- Recommendation: if you need this bag's distance, re-record with TF publication enabled in the recording launch.
