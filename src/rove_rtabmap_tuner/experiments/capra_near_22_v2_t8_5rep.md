# `capra_near_22_v2` trial 8 — n_reps=5 in-optim winner, 5-rep validation

## Why this run exists

After day-4 documented that n_reps=3 in-optim systematically biases
tail-aware scores low (MO t9 and q90 t6 both lucky-in-optim), this run
tested whether **n_reps=5 in-optim** produces honest scores that survive
5-rep validation.

## Setup

- New study `capra_near_22_v2`, `near_22` search space, q75 metric,
  **n_reps=5**, n_jobs=4, 20 trials (3 smoke + 17 scale).
- Wall time: ~5 hr total.
- Best: trial 8 (DB) at in-optim q75=0.0755 (n_reps=5 median).
- 5-rep validation with separate `ROS_DOMAIN_IDs 80-84`, run in parallel
  (5 reps simultaneously since user authorized full CPU).

## Trial 8 params (vs trial 22)

| param | trial 8 | trial 22 |
|---|---|---|
| `icp_voxel_size` | 0.034 | 0.044 |
| `icp_max_correspondence_distance` | 0.101 | 0.094 |
| `icp_iterations` | 8 | 10 |
| `icp_outlier_ratio` | 0.108 | 0.129 |
| `icp_max_translation` | 0.274 | 0.398 |
| `icp_odom_correspondence_ratio` | 0.184 | 0.159 |
| `icp_point_to_plane_k` | 33 | 27 |
| `icp_map_correspondence_ratio` | 0.086 | 0.115 |
| `odom_scan_keyframe_thr` | 0.600 | 0.722 |
| `odomf2m_scan_max_size` | 11718 | 14903 |
| `odomf2m_scan_subtract_radius` | 0.059 | 0.100 |
| `rgbd_linear_update` | 0.432 | 0.420 |
| `rgbd_angular_update` | 0.061 | 0.063 |
| `mem_stm_size` | 10 | 10 |
| `rgbd_proximity_path_max_neighbors` | 6 | 4 |

Notable: finer voxel (0.034 vs 0.044), much smaller `odom_scan_max_size`
(11718 vs 14903), tighter `odom_keyframe_thr` (0.600 — adds keyframes
more readily), shorter `subtract_radius` (0.059 vs 0.100), and more
proximity neighbors (6 vs 4).

## 5-rep validation result

| bag | r1 | r2 | r3 | r4 | r5 | median | max |
|---|---|---|---|---|---|---|---|
| moving_long_bag1 | 0.016 | 0.007 | 0.023 | 0.017 | 0.016 | 0.016 | 0.023 |
| moving_long_bag3 | 0.152 | **FAIL** | 0.097 | 0.029 | 0.115 | 0.106 | 0.152 |
| moving_long_bag4 | 0.019 | 0.040 | 0.010 | 0.012 | 0.024 | 0.019 | 0.040 |
| moving_extra_long_bag1 | 0.021 | 0.024 | 0.042 | 0.061 | 0.028 | 0.028 | 0.061 |
| moving_extra_long_bag2 | 0.020 | 0.023 | 0.010 | 0.015 | 0.032 | 0.020 | 0.032 |
| turning_bag1 | 0.080 | 0.120 | 0.124 | 0.143 | 0.215 | 0.124 | 0.215 |
| turning_bag2 | 0.132 | 0.078 | **0.370** | 0.111 | 0.045 | 0.111 | 0.370 |

**1 bag failure across 35 bag-runs (rep 2 / moving_long_bag3)**: RTAB-Map
produced `success=True` but `metrics.stats=None` — no scoreable
trajectory. The other 4 reps of bag3 had highly variable drift
(0.029-0.152) — wider per-rep spread than trial 22 (0.04-0.16) or trial
18 (0.007-0.107).

## Trial-level aggregation

```
worst-bag per rep:   0.152 / FAIL / 0.370 / 0.143 / 0.215
median worst-bag (with FAIL=1.0 penalty): 0.215
median q75 (excluding failure rep): 0.086
```

## Comparison vs known winners

| metric (5-rep median) | trial 22 | trial 18 | **v2 t8** |
|---|---|---|---|
| median q75 | 0.087 | **0.078** | 0.086 |
| median worst-bag | **0.177** | 0.207 | 0.215 (with FAIL penalty) |
| max worst-bag | **0.258** | 0.331 | **1.0 (FAIL)** |
| bag failures (35 runs) | 0 | 0 | **1** |

**Mixed result.** Trial 8's q75 is comparable to trial 22 (0.086 vs
0.087) — the n_reps=5 in-optim scoring is more honest than n_reps=3, as
expected. But trial 8 introduces a robustness issue: 1 in 5 reps had
`moving_long_bag3` fail to produce a trajectory, and the
turning_bag2 max swung to 0.370 in rep 3.

## What the n_reps=5 experiment confirms

- n_reps=5 in-optim is more honest than n_reps=3. Trial 8's in-optim
  q75=0.0755 was within 14% of its 5-rep validation q75=0.086 — much
  tighter than the trial 9 (MO, n_reps=3) gap (in-optim 0.076, 5-rep
  0.128) or q90 t6 (in-optim 0.138, 5-rep 0.232).

- However, n_reps=5 doesn't catch *rare* failure modes — bag3 only
  failed in 1 of 5 validation reps. The in-optim 5 reps happened to
  avoid the failure. To catch this, n_reps=10+ would be needed.

## Verdict

**Trial 22 remains deployment default.** Trial 8 is q75-comparable but
fails on robustness (1 bag failure). Trial 18 remains the q75-
prioritized alt with no failures.

Methodology lesson stands: n_reps=5 is *more honest* than n_reps=3 for
in-optim scoring, but **5-rep validation still needs to be a separate
step** to surface failure modes the in-optim happened to miss.

## Reproducing

Trial 8 params already listed. Standard 5-rep parallel orchestrator
(`ROS_DOMAIN_ID 80-84`).
