# MO (drift_per_path + mean_icp_ratio) study + new map metrics — diagnosis

## Background

User observed live RTAB-Map under-counting motion under `trial_6_variant_kf075`:
the robot was walking around a ~15×15 m area but the SLAM trajectory
reported tiny path lengths. The optimizer had found a degenerate solution —
minimize the ratio `drift_m / path_length_m` by shrinking the denominator.

Two responses:

1. **Multi-objective optim**: `--metric drift_per_path --metric mean_icp_ratio`.
   Hypothesis: `mean_icp_ratio` would penalize trials that game by rejecting
   scans (low icp_ratio = lots of rejection).
2. **New tracked metrics**: `map_thickness_m` (median local-plane-fit
   thickness, catches ghosting) and `cloud_spatial_extent_m` (bbox diagonal
   of assembled cloud, catches motion under-counting by comparing to path
   length).

## MO study results

`capra_mo_icpratio_v1`: near_22 search space, 10 trials, n_jobs=2, n_reps=5.
Pareto front (3 trials):

| trial | drift_per_path | mean_icp_ratio |
|---|---|---|
| 4 | 0.0496 | 0.386 |
| 5 | 0.0688 | 0.414 |
| 8 | 0.0707 | **0.457** ← highest icp_ratio |

## Deployment test: MO trial 8 (highest icp_ratio) vs trial 6 variant on the new "local" bags

|  | rosbag-local-1 | rosbag-local-2 | rosbag-local-drift |
|---|---|---|---|
| **trial_6_kf075** | **FAIL** | drift=0.038, path=175 m | drift=0.170, path=**30 m** |
| **MO trial 8** | drift=0.022, path=239 m | drift=0.027, path=180 m, icp_r=0.64 | drift=0.329, path=**12.75 m**, extent=39 m |

Two observations:

1. **MO trial 8 fixes the rosbag-local-1 failure** the variant had — real win.
   The variant's params caused RTAB-Map to fail to produce any trajectory;
   trial 8 completes with 239 m reported path (plausible for ~4.5 min walking).

2. **Trial 8 gets gamed *worse* on rosbag-local-drift**: reported 12.75 m of
   path with a cloud spanning 39 m. **Extent/path = 3.05** — a clear signature
   of motion under-counting (you cannot have a cloud larger than your traveled
   path, modulo lidar range).

## Why `mean_icp_ratio` is insufficient

`mean_icp_ratio` measures the **fraction of scan correspondences that match**
during odometry registration. It's a defense against the "reject every scan
as outlier" gaming. But it's not a defense against the "always re-anchor to
the same keyframe" gaming — that pattern can have high icp_ratio (perfect
scan alignment) and still under-count motion (because the trajectory pose
never advances, even though scans match cleanly against the stuck keyframe).

Trial 8's local-drift run: icp_ratio = 0.59 (highest of any candidate) AND
path = 12.75 m. ICP is matching cleanly — it's just matching the same
keyframe over and over.

## Right metric: cloud_spatial_extent_m vs path_length_m

Lidar scans physically cover ~5-10 m radius from each pose. For a clean
trajectory, the assembled cloud's bbox grows approximately as the trajectory
covers new ground. **You cannot have a cloud bigger than your path** (plus a
constant offset for lidar range). So:

```
extent / path > 1.5 → trajectory is under-counting motion
extent / path < 0.5 → robot walking laps in a small space (normal)
extent / path ~ 1.0 → straight-line trajectory (normal)
```

For our candidates on the new bags:

|  | rosbag-local-1 | rosbag-local-2 | rosbag-local-drift |
|---|---|---|---|
| trial 6 variant extent/path | n/a (FAIL) | 40/175 = 0.23 (ok, laps) | n/a (no extent recorded) |
| MO trial 8 extent/path | 54/239 = 0.23 | 39/180 = 0.22 | **39/12.75 = 3.05** |

Local-drift is clearly the only case where current candidates exhibit the
gaming pattern. Need an optimization run that uses extent/path as a
constraint or penalty.

## Recommended next step

A new study with **multi-objective on (drift_per_path, cloud_extent_motion_ratio)**
where `cloud_extent_motion_ratio = max(0, extent/path - 1.0)`. This adds a
penalty whenever cloud is bigger than path. Optimizer will be unable to
exploit motion under-counting without exploding this penalty.

Alternatively (simpler): a composite scalar metric like
`drift_per_path * (1 + max(0, extent/path - 1.5))`. Same intent, fits
single-objective TPE.

Implementation: ~1 hr (add the metric to scoring.py and optimizer.py).
Followed by ~5 hr study at n_jobs=2 n_reps=5.

## Notes on the implementation already committed (dfed36e)

Both `map_thickness_m` and `cloud_spatial_extent_m` are now tracked on every
trial. The `cloud_spatial_extent_m` would have flagged the variant's
local-drift result immediately had it been computed at the time. Going
forward, every trial's metrics.json contains this for diagnosis.
