# `capra_turning_v1` trial 4 — turning-only optim, 5-rep validation on 7 bags

## Hypothesis

`trial 22` (current deployment) is bottlenecked by turning bags
(`turning_bag1` median 0.107, `turning_bag2` median 0.175 in 5-rep). If we
optimize specifically for turning-bag q75, we might find params that drop
the turning floor without hurting long-bag performance.

## Setup

- New study `capra_turning_v1_smoke` on bags `turning_bag1` + `turning_bag2`
  only. `near_367` search space, `q75_drift_per_path` metric, n_reps=3,
  n_jobs=4. Seed 22.
- Optim was killed by a 30s foreground-Bash timeout, but Optuna's worker
  pool kept running past the kill via orphan processes. 7 trials made it
  to COMPLETE state (2 from smoke + 5 from scale-up).
- Best trial: **trial 4**, q75=0.1539 (turning-only n_reps=3 median).
- 5-rep validation of trial 4 on the **full 7-bag set** (long + turning),
  `ROS_DOMAIN_ID` 90-94, 22 min wall.

## Trial 4's params

| param | value | vs trial 22 |
|---|---|---|
| icp_voxel_size | 0.0314 | 0.044 (smaller) |
| icp_max_correspondence_distance | 0.0625 | 0.094 |
| icp_outlier_ratio | 0.2980 | 0.129 (stricter) |
| icp_iterations | 17 | 10 (more) |
| rgbd_linear_update | 0.2893 | 0.420 (more keyframes) |
| odom_scan_keyframe_thr | 0.7877 | 0.722 |
| mem_stm_size | 11 | 10 |

## Per-bag results (5-rep)

| bag | r1 | r2 | r3 | r4 | r5 | median | max |
|---|---|---|---|---|---|---|---|
| moving_long_bag1 | 0.007 | 0.034 | 0.012 | 0.063 | 0.019 | 0.019 | 0.063 |
| moving_long_bag3 | 0.072 | 0.103 | 0.011 | 0.072 | 0.072 | 0.072 | 0.103 |
| moving_long_bag4 | 0.034 | 0.011 | 0.012 | 0.021 | 0.018 | 0.018 | 0.034 |
| moving_extra_long_bag1 | 0.066 | 0.046 | 0.057 | 0.039 | 0.036 | 0.046 | 0.066 |
| moving_extra_long_bag2 | 0.005 | 0.042 | 0.028 | 0.012 | 0.074 | 0.028 | 0.074 |
| turning_bag1 | **0.257** | 0.111 | **0.230** | 0.122 | 0.117 | 0.122 | **0.257** |
| turning_bag2 | 0.053 | **0.131** | 0.110 | **0.432** | **0.302** | 0.131 | **0.432** |

Bold = the worst bag for that rep.

## Trial-level aggregation

```
worst-bag per rep: 0.257 / 0.131 / 0.230 / 0.432 / 0.302
median worst-bag:  0.257
max worst-bag:     0.432
q75 per rep:        0.069 / 0.107 / 0.097 / 0.092 / 0.096
median q75:         0.096
```

## Comparison vs current candidates

| metric (5-rep median) | trial 22 | **trial 4** (turning_v1) |
|---|---|---|
| median worst-bag | **0.177** | 0.257 |
| max worst-bag (5 reps) | **0.258** | 0.432 |
| median q75 | **0.087** | 0.096 |
| turning_bag1 median | 0.107 | 0.122 |
| turning_bag2 median | 0.175 | **0.131** |
| turning_bag2 max | 0.213 | **0.432** |

**Trial 4 is dominated by trial 22 on every aggregate metric.**

Per-bag picture is more nuanced:
- `turning_bag2` median: trial 4 better (0.131 vs 0.175) ← optim "worked" on the median
- `turning_bag2` max (across reps): trial 4 much worse (0.432 vs 0.213) ← variance exploded
- `turning_bag1`: trial 4 worse on both median and max

## What this tells us

Optimizing on a 2-bag training set was over-fitting. Trial 4's params hit
a lucky sample on `turning_bag2` in optim (n_reps=3 median 0.154) but the
5-rep distribution reveals **catastrophically wide variance** — drift 0.05
to 0.43 across reps. Turning bag drift is dominated by RTAB-Map
non-determinism, not by ICP params, in this region of the search space.

Trial 22's broader optimization (7 bags, q75 metric, ~24 trials) gives a
more robust answer because it integrates across multiple noise sources.

## Verdict

**Trial 22 remains the deployment winner.** Turning-only optimization
fails to generalize.

Next experiment: try `icp_force_3dof=true` on turning bags (locks
z/roll/pitch — should help pure-rotation motion). Smoke-testing now.
