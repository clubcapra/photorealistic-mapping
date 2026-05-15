# `capra_focused_v3` trial 22 — 5-rep validation

## Why this run exists

In the `capra_focused_v3` study trial 22 (q75=0.0983) came second to trial 10
(q75=0.0975) on the in-optim 3-rep median. We 5-rep-validated trial 10 first.
Trial 22 was nearly tied, but its params look quite different from trial 10's
— different operating point in q75 space. Validating it gives more confidence
in the q75 floor and tells us whether either optimum is more robust than the
other.

## Setup

- Trial 22's exact params (read from DB / trial.json — see Reproducing block).
- Same 7-bag set as trial 10's validation.
- 5 reps in parallel, `ROS_DOMAIN_ID` 80-84.
- `--max-bag-duration-s 180 --expected-update-rate 50.0`
- Wall time: ~30 min (slight slowdown from concurrent jobs earlier in the
  block; not a problem).

## Trial 22's params

| param | value | vs trial 10 |
|---|---|---|
| `icp_iterations` | 10 | 15 |
| `icp_map_correspondence_ratio` | 0.1149 | 0.0814 |
| `icp_max_correspondence_distance` | 0.0941 | 0.1437 |
| `icp_max_translation` | 0.3978 | 0.3431 |
| `icp_odom_correspondence_ratio` | 0.1587 | 0.1066 |
| `icp_outlier_ratio` | 0.1290 | 0.2710 |
| `icp_point_to_plane_k` | 27 | 23 |
| `icp_strategy` | 1 | 1 |
| `icp_voxel_size` | 0.0438 | 0.0326 |
| `mem_stm_size` | 10 | 12 |
| `odom_scan_keyframe_thr` | 0.7219 | 0.6596 |
| `odomf2m_scan_max_size` | 14903 | 11831 |
| `odomf2m_scan_subtract_radius` | 0.0996 | 0.0807 |
| `rgbd_angular_update` | 0.0631 | 0.0567 |
| `rgbd_linear_update` | 0.4200 | 0.1758 |
| `rgbd_proximity_path_max_neighbors` | 4 | 2 |

Trial 22 uses a coarser voxel (0.044), tighter ICP correspondence radius
(0.094), much stricter outlier rejection (0.13 vs 0.27 — more aggressive),
fewer ICP iterations (10 vs 15), and a much larger `rgbd_linear_update`
(0.42 vs 0.18 — fewer keyframes added per metre of motion). These are
substantively different choices from trial 10 even though both score ~0.098
on the same metric.

## Per-bag drift_per_path

| bag | r1 | r2 | r3 | r4 | r5 | median | max |
|---|---|---|---|---|---|---|---|
| moving_long_bag1 | 0.009 | 0.040 | **0.258** | 0.033 | 0.022 | 0.033 | **0.258** |
| moving_long_bag3 | 0.040 | 0.007 | 0.049 | 0.009 | 0.033 | 0.033 | 0.049 |
| moving_long_bag4 | 0.060 | 0.051 | 0.014 | 0.018 | 0.187 | 0.051 | 0.187 |
| moving_extra_long_bag1 | 0.045 | 0.041 | 0.046 | 0.068 | 0.053 | 0.046 | 0.068 |
| moving_extra_long_bag2 | 0.057 | 0.011 | 0.019 | 0.014 | 0.014 | 0.014 | 0.057 |
| turning_bag1 | 0.070 | 0.156 | 0.107 | 0.111 | 0.092 | 0.107 | 0.156 |
| turning_bag2 | 0.175 | 0.177 | **0.220** | 0.107 | 0.040 | 0.175 | **0.220** |

Bold = the worst bag for that rep.

## Trial-level aggregation

```
rep 1 max = 0.175  (turning_bag2 worst)
rep 2 max = 0.177  (turning_bag2 worst)
rep 3 max = 0.258  (moving_long_bag1 worst)
rep 4 max = 0.112  (turning_bag1 worst)
rep 5 max = 0.187  (moving_long_bag4 worst)

Median worst-bag across reps: 0.177
Max worst-bag across reps:    0.258
```

q75 per rep: 0.065, 0.103, 0.163, 0.087, 0.072 → **median q75 = 0.087**.

## Comparison: trial 22 vs trial 10 vs #367

| metric (5-rep median) | #367 | trial 10 | **trial 22** | winner |
|---|---|---|---|---|
| median worst-bag | 0.221 | 0.180 | **0.177** | trial 22 (≈tied with 10) |
| max worst-bag (5 reps) | 0.354 | 0.289 | **0.258** | trial 22 (-11% vs 10) |
| **median q75** | 0.148 | 0.130 | **0.087** | **trial 22** (-33% vs 10) |
| best-rep worst-bag | 0.157 | 0.088 | **0.112** | trial 10 |

**Trial 22 strictly beats #367. Trial 22 beats trial 10 on q75, max
worst-bag, and ties on median worst-bag.** Trial 10 has a marginally better
single-best-rep, but that's a single-rep noise advantage, not a
characteristic of the params.

The two are different operating points in the q75 floor — both ~0.10 in
optim, but trial 22's 5-rep distribution is tighter and lower on q75
specifically. Likely explanation: trial 22's `rgbd_linear_update=0.42`
(vs 10's 0.18) means fewer keyframes overall, so fewer chances for a single
bad keyframe to anchor a drift — the q75 (typical-worst-bag) penalty drops.
Trial 10's denser keyframing gives a slightly lower best-case but more
exposure to the per-rep tail.

## Updated deployment recommendation

**Trial 22 supersedes trial 10 as the recommended deployment candidate.**

Why:
- Strictly better q75 (median 0.087 vs 0.130, -33%).
- Slightly better worst-case ceiling (max 0.258 vs 0.289).
- Tied on median worst-bag.
- Smaller q75-vs-optim gap (in optim: 0.098; in 5-rep: 0.087 — *better than optim score*; trial 10 in optim: 0.097, in 5-rep: 0.130 — *significantly worse than optim score*). Suggests trial 22's optim score was honest; trial 10's was somewhat lucky.

## Reproducing

```bash
for rep in 1 2 3 4 5; do
  ROS_DOMAIN_ID=$((79+rep)) nohup ros2 run rove_rtabmap_tuner run_trial \
    --bag /home/iliana/bags/moving_long_bag1 \
    --bag /home/iliana/bags/moving_long_bag3 \
    --bag /home/iliana/bags/moving_long_bag4 \
    --bag /home/iliana/bags/moving_extra_long_bag1 \
    --bag /home/iliana/bags/moving_extra_long_bag2 \
    --bag /home/iliana/bags/turning_bag1 \
    --bag /home/iliana/bags/turning_bag2 \
    --output-root /tmp/trial_22_validation --trial-id rep_$rep \
    --expected-update-rate 50.0 --max-bag-duration-s 180 \
    --bag-play-arg=--topics --bag-play-arg=/livox/lidar --bag-play-arg=/imu/data \
    --bag-play-arg=/tf --bag-play-arg=/tf_static \
    -s icp_iterations=10 \
    -s icp_map_correspondence_ratio=0.11487825221149986 \
    -s icp_max_correspondence_distance=0.09407542611759338 \
    -s icp_max_translation=0.39775933719319057 \
    -s icp_odom_correspondence_ratio=0.15865897381314267 \
    -s icp_outlier_ratio=0.12904362151613477 \
    -s icp_point_to_plane_k=27 \
    -s icp_strategy=1 \
    -s icp_voxel_size=0.04377165263431872 \
    -s mem_stm_size=10 \
    -s odom_scan_keyframe_thr=0.7219043373627109 \
    -s odomf2m_scan_max_size=14903 \
    -s odomf2m_scan_subtract_radius=0.09963209781174709 \
    -s rgbd_angular_update=0.06309961519303404 \
    -s rgbd_linear_update=0.4199955937395548 \
    -s rgbd_proximity_path_max_neighbors=4 \
    > /tmp/trial_22_validation/rep_${rep}.log 2>&1 &
done
wait
```
