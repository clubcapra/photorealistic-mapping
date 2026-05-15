# `capra_focused_v3` winner — 5-rep validation of trial 10

> **Update (post-write):** The comparison in this doc against #367 used #367's
> *3-rep* numbers from `trial_367_validation_3rep.md`. That comparison was not
> apples-to-apples (different rep count, different bag set). A follow-up 5-rep
> run of #367 on the **same 7-bag set** lives in `baseline_367_7bag_5rep.md` —
> #367's true 5-rep median worst-bag is **0.221**, not 0.137. On apples-to-apples
> trial 10 actually **beats** #367 on every metric. The "median worst-bag"
> section below is correct for trial 10 in isolation; the comparison to #367
> should be read alongside the new doc.

## Setup

- Study: `capra_focused_v3`, search space `near_367`, metric `q75_drift_per_path`, `n_reps_per_trial=3`, `n_jobs=4`.
- 24 trials in DB (1-4 pilot, 5-7 FAIL from aborted n_jobs=3 attempt, 8-27 the n_jobs=4 batch). 21 COMPLETE.
- **Winner: DB trial id 10** (filesystem `trial_0009_rep_NN/`). Optuna n_reps=3 median q75 = **0.0975**.
- 5-rep validation: each rep on its own `ROS_DOMAIN_ID` (60-64), all 5 reps run in parallel.
- 7 bags (excluding `moving_short_bag2` and `moving_extra_long_bag4` per playbook).
- `--max-bag-duration-s 180 --expected-update-rate 50.0`
- Wall time for the 5-rep parallel run: **~22 min**.

## Trial 10's params (the ones validated)

| param | value |
|---|---|
| `icp_iterations` | 15 |
| `icp_map_correspondence_ratio` | 0.0814 |
| `icp_max_correspondence_distance` | 0.1437 |
| `icp_max_translation` | 0.3431 |
| `icp_odom_correspondence_ratio` | 0.1066 |
| `icp_outlier_ratio` | 0.2710 |
| `icp_point_to_plane_k` | 23 |
| `icp_strategy` | 1 |
| `icp_voxel_size` | 0.0326 |
| `mem_stm_size` | 12 |
| `odom_scan_keyframe_thr` | 0.6596 |
| `odomf2m_scan_max_size` | 11831 |
| `odomf2m_scan_subtract_radius` | 0.0807 |
| `rgbd_angular_update` | 0.0567 |
| `rgbd_linear_update` | 0.1758 |
| `rgbd_proximity_path_max_neighbors` | 2 |

Notable departure from #367: `icp_voxel_size` ≈ 0.033 (~60% of #367's 0.054); `icp_max_correspondence_distance` ≈ 0.144 (~1.5× #367's 0.0935); `icp_outlier_ratio` ≈ 0.27 (vs #367's 0.16); `odomf2m_scan_max_size` ≈ 11831 (vs #367's 20541, much smaller scan map).

## Per-bag drift_per_path across 5 reps

| bag | r1 | r2 | r3 | r4 | r5 | median | max |
|---|---|---|---|---|---|---|---|
| moving_long_bag1 | 0.0675 | 0.1050 | 0.0337 | 0.0550 | 0.0793 | 0.0675 | 0.1050 |
| moving_long_bag3 | 0.0597 | 0.0423 | 0.0502 | 0.1600 | 0.0581 | 0.0581 | 0.1600 |
| moving_long_bag4 | 0.0300 | 0.0307 | 0.0232 | 0.0807 | 0.0227 | 0.0300 | 0.0807 |
| moving_extra_long_bag1 | 0.0805 | 0.1364 | **0.2892** | 0.0928 | 0.1478 | 0.1364 | **0.2892** |
| moving_extra_long_bag2 | 0.0446 | 0.0341 | 0.0086 | 0.0937 | 0.0125 | 0.0341 | 0.0937 |
| turning_bag1 | 0.0787 | 0.1188 | 0.1350 | **0.1801** | 0.1123 | 0.1188 | **0.1801** |
| turning_bag2 | 0.0883 | 0.1167 | **0.2134** | 0.1532 | **0.2070** | 0.1532 | 0.2134 |

Bold entries = the worst bag for that rep.

## Trial-level aggregation

```
rep 1 max_drift_per_path = 0.0883  (turning_bag2 worst)
rep 2 max_drift_per_path = 0.1364  (moving_extra_long_bag1 worst)
rep 3 max_drift_per_path = 0.2892  (moving_extra_long_bag1 worst)
rep 4 max_drift_per_path = 0.1801  (turning_bag1 worst)
rep 5 max_drift_per_path = 0.2070  (turning_bag2 worst)

Median worst-bag across reps: 0.1801
Max worst-bag across reps:    0.2892
```

q75 per rep (linear-interp 75th percentile across 7 bags):
0.080, 0.118, 0.174, 0.157, 0.130 → median q75 = **0.130**.

## Comparison vs trial #367 (3-rep validation, same metric scheme)

| metric | trial #367 (3 reps) | trial 10 (5 reps) | winner |
|---|---|---|---|
| median worst-bag drift_per_path | **0.137** | 0.180 | **#367** |
| q75 drift_per_path (single-rep) | ~0.20 | **0.130** | **trial 10** |
| best-rep worst-bag | 0.042 | 0.088 | **#367** |
| worst-rep worst-bag | 0.274 | 0.289 | comparable |

(#367's q75 is back-of-envelope from the rep1/rep2/rep3 tables in `trial_367_validation_3rep.md`, not a measured aggregate.)

**Trial 10 is genuinely better on q75 (typical-worst-bag) — that's what it was optimized for — but #367 still wins on max-aggregation (absolute-worst-bag).** This is the expected trade-off between q75 and max metrics: q75 sacrifices some absolute robustness for better typical performance.

## Per-bag stability vs #367

Wide-variance bags (where trial 10's non-determinism dominates):
- `moving_extra_long_bag1`: **0.080-0.289** (3.6× spread). #367 had 0.035-0.048 — much tighter. Trial 10 is *less stable* on this bag.
- `turning_bag2`: 0.088-0.213 (2.4× spread). #367 had 0.042-0.274 — similar instability.

Tight-variance bags (where trial 10 is very stable):
- `moving_long_bag4`: 0.023-0.081 (3.5× but small absolute). Best-in-class.
- `moving_extra_long_bag2`: 0.009-0.094 (10× spread but very low). Better than #367 (#367: 0.023-0.108).

## Bottom line

Trial 10 (`capra_focused_v3` winner) is:
- **Strictly better than #367 on typical-bag performance** (q75 = 0.13 vs ~0.20).
- **Strictly worse than #367 on worst-case robustness** (median worst-bag = 0.180 vs 0.137).
- **Roughly comparable on worst-rep behavior** (~0.29 ceiling).

The two optima represent different operating points in the q75 ↔ max trade-off. **For deployment, the choice depends on whether you care more about average-case loop closure (trial 10) or worst-case loss-of-tracking avoidance (#367).** Neither dominates the other.

Suggested next step (not yet done): re-tune with `--metric max_drift_per_path --n-reps-per-trial 3` in the same `near_367` space, see if there's a third optimum that beats #367 on max while keeping trial 10's q75 gains. Optuna sees a different objective there, may find different params.

## Reproducing

Trial 10's params already listed above. To re-run the 5-rep validation:

```bash
for rep in 1 2 3 4 5; do
  ROS_DOMAIN_ID=$((59+rep)) nohup ros2 run rove_rtabmap_tuner run_trial \
    --bag /home/iliana/bags/moving_long_bag1 \
    --bag /home/iliana/bags/moving_long_bag3 \
    --bag /home/iliana/bags/moving_long_bag4 \
    --bag /home/iliana/bags/moving_extra_long_bag1 \
    --bag /home/iliana/bags/moving_extra_long_bag2 \
    --bag /home/iliana/bags/turning_bag1 \
    --bag /home/iliana/bags/turning_bag2 \
    --output-root /tmp/top_trial_validation --trial-id rep_$rep \
    --expected-update-rate 50.0 --max-bag-duration-s 180 \
    --bag-play-arg=--topics --bag-play-arg=/livox/lidar --bag-play-arg=/imu/data \
    --bag-play-arg=/tf --bag-play-arg=/tf_static \
    -s icp_iterations=15 \
    -s icp_map_correspondence_ratio=0.08135050897706239 \
    -s icp_max_correspondence_distance=0.14374396367206072 \
    -s icp_max_translation=0.3431394208139219 \
    -s icp_odom_correspondence_ratio=0.10659094630484416 \
    -s icp_outlier_ratio=0.2710250798581572 \
    -s icp_point_to_plane_k=23 \
    -s icp_strategy=1 \
    -s icp_voxel_size=0.032556530204107516 \
    -s mem_stm_size=12 \
    -s odom_scan_keyframe_thr=0.6595928997078367 \
    -s odomf2m_scan_max_size=11831 \
    -s odomf2m_scan_subtract_radius=0.08068525784128154 \
    -s rgbd_angular_update=0.05667141599421128 \
    -s rgbd_linear_update=0.17581379840891886 \
    -s rgbd_proximity_path_max_neighbors=2 \
    > /tmp/top_trial_validation/rep_${rep}.log 2>&1 &
done
wait
```
