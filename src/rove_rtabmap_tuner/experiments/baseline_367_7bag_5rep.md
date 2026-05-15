# Baseline #367 — 5-rep validation on the 7-bag set (apples-to-apples vs trial 10)

## Why this run exists

The previous `trial_367_validation_3rep.md` used 3 reps on a different bag set
(9 bags including `moving_short_bag2` and `moving_extra_long_bag4`). The
`capra_focused_v3_winner_5rep.md` doc compared trial 10's 5-rep median (0.180)
against #367's 3-rep median (0.137) — but those numbers were on different bag
sets *and* different rep counts. The conclusion ("#367 still wins on
max-aggregation") was therefore not apples-to-apples.

This run re-runs #367 with 5 reps on the **same 7-bag set** used for trial 10's
validation, so the two can be compared cleanly.

## Setup

- Trial #367's exact params (from `TUNING_PLAYBOOK.md`).
- 7 bags: `moving_long_bag1, moving_long_bag3, moving_long_bag4,
  moving_extra_long_bag1, moving_extra_long_bag2, turning_bag1, turning_bag2`
  (no `moving_short_bag2`, no `moving_extra_long_bag4`).
- 5 reps in parallel, each on its own `ROS_DOMAIN_ID` 75-79.
- `--max-bag-duration-s 180 --expected-update-rate 50.0`
- Wall time: ~22 min.

## Per-bag drift_per_path

| bag | r1 | r2 | r3 | r4 | r5 | median | max |
|---|---|---|---|---|---|---|---|
| moving_long_bag1 | 0.186 | 0.130 | 0.042 | **0.354** | 0.157 | 0.157 | **0.354** |
| moving_long_bag3 | 0.038 | 0.074 | 0.036 | 0.017 | 0.105 | 0.038 | 0.105 |
| moving_long_bag4 | 0.043 | 0.038 | 0.046 | 0.066 | 0.048 | 0.046 | 0.066 |
| moving_extra_long_bag1 | 0.064 | 0.079 | 0.105 | 0.058 | 0.065 | 0.065 | 0.105 |
| moving_extra_long_bag2 | 0.047 | 0.023 | 0.029 | 0.020 | 0.024 | 0.024 | 0.047 |
| turning_bag1 | 0.106 | 0.190 | **0.221** | 0.165 | 0.130 | 0.165 | **0.221** |
| turning_bag2 | 0.163 | **0.230** | 0.220 | 0.131 | 0.047 | 0.163 | **0.230** |

Bold = worst-bag for that rep.

## Trial-level aggregation

```
rep 1 max = 0.186  (moving_long_bag1 worst)
rep 2 max = 0.230  (turning_bag2 worst)
rep 3 max = 0.221  (turning_bag1 worst)
rep 4 max = 0.354  (moving_long_bag1 worst)
rep 5 max = 0.157  (moving_long_bag1 worst)

Median worst-bag across reps: 0.221
Max worst-bag across reps:    0.354
```

q75 per rep (linear-interp across 7 bags):
0.134, 0.160, 0.163, 0.148, 0.117 → median q75 = **0.148**.

## Comparison vs trial 10 (5-rep, same 7 bags, same methodology)

| metric (5-rep median) | #367 | capra_focused_v3 trial 10 | winner |
|---|---|---|---|
| **median worst-bag** | 0.221 | **0.180** | **trial 10** (-18%) |
| **median q75** | 0.148 | **0.130** | **trial 10** (-12%) |
| best-rep worst-bag | 0.157 | 0.088 | **trial 10** |
| worst-rep worst-bag | **0.354** | 0.289 | **trial 10** (lower ceiling) |
| max worst-bag (5 reps) | 0.354 | 0.289 | **trial 10** |

**Trial 10 strictly dominates #367 on this bag set.** No metric favors #367.

## Where #367's earlier "0.137 median" came from

The original `trial_367_validation_3rep.md` reported median worst-bag = 0.137
after excluding `moving_short_bag2`. That measurement:
- Was 3 reps, not 5 — so smaller sample.
- Included `moving_extra_long_bag4` in the bag list (it failed to produce a
  trajectory and was dropped, but this changed which bags were scored).
- Per-rep worst-bag (without bag2): 0.042 / 0.137 / 0.274.

In the new 5-rep run on a tighter bag set, the worst-bag values per rep are
0.157 / 0.230 / 0.221 / 0.354 / 0.157 — systematically higher. The 0.137 was
lucky; the honest figure is closer to 0.22.

Same story applies in interpretation: RTAB-Map's non-determinism produces
larger per-rep swings than 3 reps can characterize, biasing 3-rep medians low.

## Implications

1. **Trial 10 (capra_focused_v3 winner) is the better deployment candidate
   than #367 on this bag set.** Lower median worst-bag (0.180 vs 0.221), lower
   median q75 (0.130 vs 0.148), and lower max worst-bag (0.289 vs 0.354).

2. **Update `TUNING_PLAYBOOK.md`** to reflect trial 10's superiority on
   apples-to-apples comparison. #367 stays in the doc as the
   historically-cited optimum but trial 10 is the new recommended deployment.

3. **`moving_long_bag1` is the dominant noise source for #367** — drift
   varies 0.042 → 0.354 across reps. Trial 10's `moving_long_bag1`: 0.034 →
   0.105 (much tighter). One reason trial 10 wins on max: it's more stable on
   the noisy bag.

4. **`turning_bag2` remains a structural challenge for both** — 0.05 to 0.23
   for #367, 0.09 to 0.21 for trial 10. The turning motion seems hard to keep
   bounded under any params.

## Reproducing

```bash
for rep in 1 2 3 4 5; do
  ROS_DOMAIN_ID=$((74+rep)) nohup ros2 run rove_rtabmap_tuner run_trial \
    --bag /home/iliana/bags/moving_long_bag1 \
    --bag /home/iliana/bags/moving_long_bag3 \
    --bag /home/iliana/bags/moving_long_bag4 \
    --bag /home/iliana/bags/moving_extra_long_bag1 \
    --bag /home/iliana/bags/moving_extra_long_bag2 \
    --bag /home/iliana/bags/turning_bag1 \
    --bag /home/iliana/bags/turning_bag2 \
    --output-root /tmp/baseline_367_7bag_validation --trial-id rep_$rep \
    --expected-update-rate 50.0 --max-bag-duration-s 180 \
    --bag-play-arg=--topics --bag-play-arg=/livox/lidar --bag-play-arg=/imu/data \
    --bag-play-arg=/tf --bag-play-arg=/tf_static \
    -s icp_voxel_size=0.054 -s icp_max_correspondence_distance=0.0935 \
    -s icp_iterations=15 -s icp_outlier_ratio=0.164 -s icp_max_translation=0.345 \
    -s icp_point_to_plane_k=27 -s icp_strategy=1 \
    -s icp_odom_correspondence_ratio=0.146 -s icp_map_correspondence_ratio=0.119 \
    -s odom_scan_keyframe_thr=0.836 -s odomf2m_scan_max_size=20541 \
    -s odomf2m_scan_subtract_radius=0.0554 -s rgbd_linear_update=0.288 \
    -s rgbd_angular_update=0.077 -s rgbd_proximity_path_max_neighbors=3 \
    -s mem_stm_size=12 \
    > /tmp/baseline_367_7bag_validation/rep_${rep}.log 2>&1 &
done
wait
```
