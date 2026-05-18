# `capra_near_22_v1` trial 18 — 5-rep validation (Pareto alternative)

## Why this run exists

`SEARCH_SPACE_NEAR_22` was added in commit 3218065 to refine on trial 22's
neighborhood. After 20 trials of TPE optimization, **trial 18 won at
q75=0.0845 (in-optim, n_reps=3 median)** — slightly better than trial 22's
in-optim 0.0975. This 5-rep validation tests whether it holds up.

## Setup

- Trial 18's exact params (read from `study_near_22_v1` DB).
- 7-bag set, 5 reps in parallel batches of 2 (CPU-aware), 3 batches.
- `ROS_DOMAIN_ID 85-89`.
- Wall time: ~30 min.

## Trial 18's params (vs trial 22)

| param | trial 18 | trial 22 |
|---|---|---|
| `icp_iterations` | 7 | 10 |
| `icp_map_correspondence_ratio` | 0.103 | 0.115 |
| `icp_max_correspondence_distance` | 0.061 | 0.094 |
| `icp_max_translation` | 0.361 | 0.398 |
| `icp_odom_correspondence_ratio` | 0.138 | 0.159 |
| `icp_outlier_ratio` | 0.113 | 0.129 |
| `icp_point_to_plane_k` | 28 | 27 |
| `icp_voxel_size` | 0.049 | 0.044 |
| `mem_stm_size` | 10 | 10 |
| `odom_scan_keyframe_thr` | 0.503 | 0.722 |
| `odomf2m_scan_max_size` | 14139 | 14903 |
| `odomf2m_scan_subtract_radius` | 0.100 | 0.100 |
| `rgbd_angular_update` | 0.044 | 0.063 |
| `rgbd_linear_update` | 0.467 | 0.420 |
| `rgbd_proximity_path_max_neighbors` | 3 | 4 |

Trial 18 is in trial 22's neighborhood — close on most params. Notable
differences: fewer ICP iterations (7 vs 10), tighter ICP correspondence
distance (0.061 vs 0.094 — almost 1.5× tighter), and lower
`odom_scan_keyframe_thr` (0.503 vs 0.722 — adds keyframes more readily).

## Per-bag 5-rep results

| bag | r1 | r2 | r3 | r4 | r5 | median | max |
|---|---|---|---|---|---|---|---|
| moving_long_bag1 | 0.023 | 0.015 | 0.034 | 0.035 | 0.031 | 0.031 | 0.035 |
| moving_long_bag3 | 0.107 | 0.023 | 0.025 | 0.023 | 0.092 | 0.025 | 0.107 |
| moving_long_bag4 | 0.042 | 0.051 | 0.029 | 0.014 | 0.025 | 0.029 | 0.051 |
| moving_extra_long_bag1 | 0.004 | 0.074 | 0.051 | 0.042 | 0.062 | 0.051 | 0.074 |
| moving_extra_long_bag2 | 0.124 | 0.004 | 0.041 | 0.009 | 0.037 | 0.037 | 0.124 |
| turning_bag1 | **0.174** | **0.208** | **0.207** | **0.127** | 0.064 | **0.174** | **0.208** |
| turning_bag2 | **0.196** | 0.071 | 0.148 | 0.088 | **0.331** | 0.148 | **0.331** |

## Trial-level aggregation

```
worst-bag per rep: 0.196 / 0.208 / 0.207 / 0.127 / 0.331
median worst-bag:  0.207
max worst-bag:     0.331
q75 per rep:        0.149 / 0.072 / 0.099 / 0.065 / 0.078
median q75:         0.078
```

## Comparison vs trial 22 (current deployment winner)

| metric (5-rep median) | trial 22 | **trial 18** | delta |
|---|---|---|---|
| median worst-bag | 0.177 | 0.207 | **trial 22 wins** (-14% better) |
| **median q75** | 0.087 | **0.078** | **trial 18 wins** (-10% better) |
| max worst-bag (5 reps) | 0.258 | 0.331 | **trial 22 wins** (-22% better) |
| best-rep worst-bag | 0.088 | 0.127 | trial 22 wins |
| worst-rep worst-bag | 0.289 | 0.331 | trial 22 wins |

**Trial 18 is a Pareto alternative, not a dominating winner.**

- **Trial 18 has lower median q75** — better typical-bag performance.
- **Trial 22 has tighter worst-bag distribution** — better worst-case
  robustness.

The two operating points sit on a Pareto front in q75 vs max-aggregation.

## Per-bag interpretation

- **Long bags**: Trial 18 is consistently better — all 5 long bags have
  lower median drift than under trial 22. Wait — comparing directly:

  | bag | trial 22 median | trial 18 median |
  |---|---|---|
  | moving_long_bag1 | 0.068 | 0.031 ← trial 18 better |
  | moving_long_bag3 | 0.058 | 0.025 ← trial 18 better |
  | moving_long_bag4 | 0.030 | 0.029 ← tied |
  | moving_extra_long_bag1 | 0.136 | 0.051 ← trial 18 better |
  | moving_extra_long_bag2 | 0.034 | 0.037 ← tied |

  **Trial 18 wins decisively on long bags** — median drift roughly halved
  on `moving_long_bag1`, `moving_long_bag3`, and `moving_extra_long_bag1`.

- **Turning bags**: Trial 22 is more stable.

  | bag | trial 22 median | trial 18 median |
  |---|---|---|
  | turning_bag1 | 0.107 | 0.174 ← trial 22 better |
  | turning_bag2 | 0.175 | 0.148 ← trial 18 better |
  | turning_bag1 max | 0.156 | 0.208 ← trial 22 better |
  | turning_bag2 max | 0.213 | 0.331 ← trial 22 better |

  **Trial 22 wins on turning-bag robustness** (lower max especially).

## Deployment recommendation

**Default**: keep `trial 22` as the deployment winner. Better worst-case
robustness, smaller q75↔max gap.

**For long-bag-dominant deployments** (paths primarily forward motion,
minimal pure rotation): **trial 18 is better** — long bags scoring ~2×
tighter. If the deployment knows it won't see tight rotations in
practice, trial 18 reduces typical-bag drift substantially.

The `near_22` SEARCH_SPACE add (commit 3218065) is now justified by this
finding — TPE found a genuine alternative operating point within ±30-40%
of trial 22 that's better on long bags.

## Updated standings (5-rep medians on 7-bag set)

| candidate | median worst-bag | median q75 | max worst-bag | recommended for |
|---|---|---|---|---|
| **trial 22** | **0.177** | 0.087 | **0.258** | overall winner (default) |
| **trial 18 (near_22 v1)** | 0.207 | **0.078** | 0.331 | long-bag-dominant |
| trial 10 | 0.180 | 0.130 | 0.289 | runner-up |
| #367 | 0.221 | 0.148 | 0.354 | historical baseline |
| trial 349 | 0.233 | 0.099 | 0.414 | long-only specialist |

## Reproducing

Trial 18's params already listed. Standard 5-rep launch loop (orchestrator
pattern from earlier writeups, 2 parallel reps × 3 batches with
`ROS_DOMAIN_ID 85-89`).
