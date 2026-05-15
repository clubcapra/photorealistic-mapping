# `icp_force_3dof` / `icp_force_4dof` — smoke tests (negative result)

## Hypothesis

Trial 22 (current deployment winner) is bottlenecked by turning bags
(`turning_bag1` median 0.107, `turning_bag2` median 0.175 in 5-rep, with
some reps swinging to 0.43). If turning motion is causing roll/pitch
spurious ICP fits, locking those DoFs should help.

## Setup

- Single-rep smoke on `turning_bag1` + `turning_bag2`.
- Base: trial 22's params, with one flag flipped.
- `ROS_DOMAIN_ID 99` (3DoF), `98` (4DoF).
- `--max-bag-duration-s 180 --expected-update-rate 50.0`.

## Results

| config | turning_bag1 drift/path | turning_bag2 drift/path |
|---|---|---|
| trial 22 baseline (5-rep median) | **0.107** | **0.175** |
| trial 22 + `icp_force_3dof=true` (1 rep) | 0.162 | 0.316 |
| trial 22 + `icp_force_4dof=true` (1 rep) | 0.376 | 0.248 |

Both DoF locks make things **substantially worse**, especially 4DoF on
`turning_bag1` (+250%). Single-rep evidence but the gap is large enough
to be real signal.

## Why

The pipeline already has:
- Lidar deskewing (`lidar_deskewing`) consuming IMU to compensate motion
  during a scan.
- ICP odometry with IMU init (`-r imu:=/imu/data`) — the IMU provides the
  initial guess for ICP each frame, including pitch/roll.
- A short-term memory of recent keyframes for graph-side optimization.

Locking ICP to (x, y, yaw) or (x, y, z, yaw) removes degrees of freedom
that the IMU integration has already correctly fed back into the
trajectory. ICP can no longer correct for small calibration errors or
sensor noise in pitch/roll, so per-scan residuals accumulate.

The robot is on uneven floors (the bags include door thresholds and
gentle slopes), and 6-DoF ICP correctly handles these. Forcing 3DoF/4DoF
treats them as planar, which they're not.

## Verdict

`icp_force_3dof` and `icp_force_4dof` are not useful for this lidar+IMU
stack on this set of bags. They might be useful on a different
configuration (pure lidar without IMU, perfectly planar ground), but not
ours.

Keep both flags as opt-in via `--set` in the template (already done) but
do not add to `SEARCH_SPACE`. They won't help auto-tuning.

## Next experiment

Pivot to wider-space exploration: `--search-space wide` (the original
16-dim) with `q75_drift_per_path` metric. Trial 22 came from `near_367`
which is narrow; wide space may have a different optimum.
