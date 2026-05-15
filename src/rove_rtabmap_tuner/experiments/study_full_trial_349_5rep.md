# study_full trial #349 — 5-rep validation on 7-bag set

## Why this run exists

In the apples-to-apples discussion (`baseline_367_7bag_5rep.md` and
`trial_22_5rep.md`), the question came up: is trial 22 actually better than
the top trials from the larger `study_full` (`capra_full_v1`, 717 trials)?
Naive comparison favored study_full's #349 — but those scores were
single-rep, while trial 22's were 5-rep medians.

This run answers the question by re-running #349 with 5 reps on the same
7-bag set used for all the recent winners.

## Setup

- Trial #349's params from `study_full/optuna.db` (read-only).
- 7 bags: 5 long bags + 2 turning bags.
- 5 reps in parallel, `ROS_DOMAIN_ID` 85-89.
- `--max-bag-duration-s 180 --expected-update-rate 50.0`
- Wall time: ~22 min.

## Per-bag drift_per_path

| bag | r1 | r2 | r3 | r4 | r5 | median | max |
|---|---|---|---|---|---|---|---|
| moving_long_bag1 | 0.045 | 0.040 | 0.013 | 0.024 | 0.167 | 0.040 | 0.167 |
| moving_long_bag3 | 0.033 | 0.006 | 0.021 | 0.046 | 0.042 | 0.033 | 0.046 |
| moving_long_bag4 | 0.034 | 0.015 | 0.007 | 0.016 | 0.017 | 0.016 | 0.034 |
| moving_extra_long_bag1 | 0.164 | 0.047 | 0.043 | 0.056 | 0.128 | 0.056 | 0.164 |
| moving_extra_long_bag2 | 0.034 | 0.024 | 0.009 | 0.015 | 0.022 | 0.022 | 0.034 |
| turning_bag1 | 0.079 | 0.111 | 0.155 | 0.141 | 0.130 | 0.130 | 0.155 |
| turning_bag2 | **0.233** | **0.153** | **0.247** | 0.044 | **0.414** | **0.233** | **0.414** |

Bold = the worst bag for that rep. `turning_bag2` was worst in 4 of 5 reps.

## Trial-level aggregation

7-bag set (deployment-relevant):
```
worst-bag per rep: 0.233 / 0.153 / 0.247 / 0.141 / 0.414
median worst-bag: 0.233
max worst-bag:    0.414
q75 per rep:       0.121 / 0.079 / 0.099 / 0.051 / 0.148
median q75:        0.099
```

5-bag long-only subset (apples-to-apples with study_full):
```
max worst-bag (5 reps): 0.164 / 0.047 / 0.043 / 0.056 / 0.167
median max:             0.056
median q75:             0.045
```

## Verdict

**Trial 349 is NOT a lucky one-shot — its 5-bag long performance matches
study_full's reported single-rep number (median max 0.056 vs reported
0.055).** That's strong evidence the original study_full pipeline produced
genuine signal on the long-bag set it was tuned against.

**BUT trial 349 is bad on turning bags** — `turning_bag2` was worst-bag in
4 of 5 reps with drift 0.04 → 0.41, dominating the 7-bag worst-bag
aggregation. This is unsurprising because:

- `study_full` (`capra_full_v1`) was run before the turning bags existed in
  the eval set. Its trials never saw turning motion during optimization.
- Trial 349's params (`icp_max_translation=1.59` — very permissive, vs
  trial 22's 0.40) likely match the long-trajectory motion pattern but
  drift on tight turns.

## Comparison vs current candidates (7-bag, 5-rep medians)

| metric | #367 | trial 10 | **trial 22** | trial 349 |
|---|---|---|---|---|
| median worst-bag | 0.221 | 0.180 | **0.177** | 0.233 |
| max worst-bag (5 reps) | 0.354 | 0.289 | **0.258** | **0.414** |
| median q75 | 0.148 | 0.130 | **0.087** | 0.099 |

**Trial 22 remains the deployment winner on the 7-bag set.** Trial 349 is
the long-bag specialist — useful as a candidate if the deployment workload
has minimal turning.

## Implication for tuning strategy

The "per-motion-pattern cluster" observation in `TUNING_PLAYBOOK.md`
predicts this outcome: long bags and turning bags want different params.
Trial 349 (long-only optimum) is dominated by trial 22 (mixed-motion
optimum) when turning is present.

Suggested follow-up (not yet run): tune specifically for turning bags
(`turning_bag1` + `turning_bag2` only, q75 metric) to find a third
operating point. Then either:
- (a) Deploy per-motion-pattern selection: classify the workload at
  runtime, switch params.
- (b) Multi-objective Pareto: optimize for both long-bag and
  turning-bag q75 simultaneously, find a balance point.
- (c) Manual: ensemble of #349 (long) and turning-specialist params,
  selected per-bag.

## Reproducing

Trial 349's params (16 — read from study_full DB):
```
icp_iterations=15
icp_map_correspondence_ratio=0.45707203316347306
icp_max_correspondence_distance=0.09349116534947367
icp_max_translation=1.5928965965217385
icp_odom_correspondence_ratio=0.14557703958105986
icp_outlier_ratio=0.16418695240346332
icp_point_to_plane_k=20
icp_strategy=1
icp_voxel_size=0.05393734055942647
mem_stm_size=3
odom_scan_keyframe_thr=0.7578565856666127
odomf2m_scan_max_size=20541
odomf2m_scan_subtract_radius=0.05541131615803816
rgbd_angular_update=0.07679603952087627
rgbd_linear_update=0.28831303876515196
rgbd_proximity_path_max_neighbors=2
```

Identical 5-rep launch loop as `trial_22_5rep.md`, just substitute these
params and use `ROS_DOMAIN_ID 85-89` and output dir
`/tmp/trial_349_validation`.
