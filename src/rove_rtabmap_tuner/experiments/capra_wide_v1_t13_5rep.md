# `capra_wide_v1` trial 13 — wide-search winner 5-rep validation (negative)

## Hypothesis

Trial 22 won in `near_367` — a narrow ±30-50% region around #367's params.
Maybe there's a better operating point further away. Try wide-search.

## Setup

- Study `capra_wide_v1_smoke`, search space `wide` (16-dim, original ranges).
- 17 trials total: 3 smoke (n_reps=2) + 14 scale-up (n_reps=3). 13 COMPLETE,
  4 FAIL (RTAB-Map failed to produce a scoreable trajectory for some param
  combos in wide space).
- Best wide trial: **#13**, in-optim q75=0.135 (n_reps=3 median).
- 5-rep validation on 7-bag set, 2 parallel reps at a time (CPU-aware per
  user request for ≥2 free cores), 3 batches.
- Wall time: ~30 min validation.

## Trial 13's params (vs trial 22)

| param | trial 13 | trial 22 |
|---|---|---|
| `icp_voxel_size` | 0.074 | 0.044 |
| `icp_max_correspondence_distance` | **0.514** | 0.094 |
| `icp_iterations` | 8 | 10 |
| `icp_outlier_ratio` | 0.216 | 0.129 |
| `icp_max_translation` | 0.542 | 0.398 |
| `mem_stm_size` | 12 | 10 |
| `odom_scan_keyframe_thr` | 0.245 | 0.722 |
| `rgbd_angular_update` | 0.461 | 0.063 |
| `rgbd_linear_update` | 0.064 | 0.420 |
| `rgbd_proximity_path_max_neighbors` | 9 | 4 |

Trial 13 is a very different operating point: huge correspondence radius,
much more aggressive keyframing (rgbd_linear_update 7× tighter).

## 5-rep per-bag results

| bag | r1 | r2 | r3 | r4 | r5 | median | max |
|---|---|---|---|---|---|---|---|
| moving_long_bag1 | 0.205 | 0.244 | 0.130 | 0.128 | 0.109 | 0.130 | 0.244 |
| moving_long_bag3 | 0.274 | 0.066 | 0.176 | 0.067 | 0.120 | 0.120 | 0.274 |
| moving_long_bag4 | 0.128 | 0.134 | 0.001 | 0.093 | 0.001 | 0.093 | 0.134 |
| moving_extra_long_bag1 | 0.101 | 0.074 | 0.203 | 0.064 | 0.126 | 0.101 | 0.203 |
| moving_extra_long_bag2 | 0.124 | 0.127 | 0.028 | 0.097 | 0.049 | 0.097 | 0.127 |
| turning_bag1 | 0.095 | 0.098 | 0.231 | 0.041 | 0.299 | 0.098 | 0.299 |
| turning_bag2 | 0.288 | 0.104 | 0.023 | 0.204 | 0.382 | 0.204 | 0.382 |

## Trial-level

```
worst-bag per rep: 0.288, 0.244, 0.231, 0.204, 0.382
median worst-bag:  0.244
max worst-bag:     0.382
q75 per rep:        0.197, 0.182, 0.224, 0.151, 0.234
median q75:        0.190
```

## Comparison

| metric (5-rep median) | trial 22 | trial 10 | trial 349 | **trial 13** |
|---|---|---|---|---|
| median worst-bag | **0.177** | 0.180 | 0.233 | **0.244** |
| median q75 | **0.087** | 0.130 | 0.099 | **0.190** |
| max worst-bag (5 reps) | **0.258** | 0.289 | 0.414 | 0.382 |

**Trial 13 is strictly worse than trial 22 on every metric.** Wide search
did not find a better operating point. Even trial 10 (the prior runner-up
in `near_367`) is comparable; trial 13 is clearly worse than trial 10.

## Why wide-search failed

1. **TPE was given too few trials in a too-wide space** (16 trials in
   16-dim wide ranges ≈ 1 sample per dim). Wide-space search needs
   substantially more trials to converge.

2. **The `near_367` basin is genuinely good.** Trial 22 and trial 10
   sit close to #367's neighborhood and beat any wide-search trial.
   When the prior optimum is informative, narrow exploration around it
   beats wide random exploration with the same trial budget.

3. **Wide-space FAIL rate**: 4 of 18 trials (22%) failed by triggering
   RTAB-Map's sparsity / min-poses rejection. The wide ranges include
   combinations that produce too few keyframes for scoring. In
   `near_367`, almost all trials produce valid trajectories.

## Verdict

**Stop wide-search.** Future tuning should stay in `near_367` or
narrower (a hypothetical `near_22` would be even better).

**Trial 22 remains the deployment winner** after extensive search around
it: 24 trials in `near_367` (capra_focused_v3), 10 trials with
max-metric (capra_max_v1), 7 trials turning-only (capra_turning_v1),
and 17 trials wide (capra_wide_v1). None of these improvements stuck.

Updated final standings (5-rep medians on 7-bag set):

| candidate | median worst-bag | median q75 | max worst-bag | scope |
|---|---|---|---|---|
| **trial 22** | **0.177** | **0.087** | **0.258** | overall winner |
| trial 10 | 0.180 | 0.130 | 0.289 | runner-up |
| trial 349 | 0.233 | 0.099 | 0.414 | long-bag specialist |
| #367 | 0.221 | 0.148 | 0.354 | historical baseline |
| trial 13 (wide) | 0.244 | 0.190 | 0.382 | wide-search loss |
| turning_v1 t4 | 0.257 | 0.096 | 0.432 | turning-only over-fit |

## Reproducing

Trial 13's params already listed above. Standard 5-rep launch loop as in
`trial_22_5rep.md`.
