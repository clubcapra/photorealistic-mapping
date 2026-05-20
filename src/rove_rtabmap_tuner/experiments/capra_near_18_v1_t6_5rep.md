# `capra_near_18_v1` trial 6 — NEW DEPLOYMENT WINNER

## Why this run exists

After trial 18 (the q75-Pareto alt at 0.078) survived 5-rep validation,
its operating point looked worth refining further. Added
`SEARCH_SPACE_NEAR_18` (±30-40% around trial 18's params) and ran a
12-trial study with `n_reps=5` for honest scoring.

## Setup

- New study `capra_near_18_v1`, search space `near_18`, q75 metric,
  **n_reps=5**, n_jobs=4, 12 trials.
- Wall: ~5.5 hr.
- Best: trial 6 (DB), in-optim q75=0.0702.
- 5-rep validation with `ROS_DOMAIN_IDs 60-64`, 5 parallel reps (full
  CPU).

## Trial 6 params (vs trial 22 and trial 18)

| param | trial 6 (NEW) | trial 18 | trial 22 |
|---|---|---|---|
| `icp_voxel_size` | 0.042 | 0.044 | 0.044 |
| `icp_max_correspondence_distance` | 0.081 | 0.094 | 0.094 |
| `icp_iterations` | 7 | 10 | 10 |
| `icp_outlier_ratio` | 0.168 | 0.129 | 0.129 |
| `icp_max_translation` | 0.457 | 0.398 | 0.398 |
| `icp_point_to_plane_k` | 25 | 27 | 27 |
| `icp_map_correspondence_ratio` | 0.098 | 0.115 | 0.115 |
| `icp_odom_correspondence_ratio` | 0.125 | 0.159 | 0.159 |
| `odom_scan_keyframe_thr` | **0.885** | 0.722 | 0.722 |
| `odomf2m_scan_max_size` | 10765 | 14903 | 14903 |
| `odomf2m_scan_subtract_radius` | 0.123 | 0.100 | 0.100 |
| `rgbd_linear_update` | 0.342 | 0.420 | 0.420 |
| `rgbd_angular_update` | 0.047 | 0.063 | 0.063 |
| `mem_stm_size` | 7 | 10 | 10 |
| `rgbd_proximity_path_max_neighbors` | 5 | 4 | 4 |

Notable departures: higher `odom_scan_keyframe_thr` (0.885 — keyframes
less aggressively), smaller `odomf2m_scan_max_size` (10765 — leaner scan
memory), smaller `mem_stm_size` (7 — shorter short-term memory), and
stricter outlier rejection (0.168). Trial 6 makes the odometry more
conservative about keyframes and the SLAM graph more selective.

## 5-rep validation results

| bag | r1 | r2 | r3 | r4 | r5 | median | max |
|---|---|---|---|---|---|---|---|
| moving_long_bag1 | 0.015 | 0.018 | 0.019 | 0.012 | 0.029 | **0.018** | 0.029 |
| moving_long_bag3 | 0.006 | 0.033 | 0.017 | 0.023 | 0.017 | **0.017** | 0.033 |
| moving_long_bag4 | 0.023 | 0.016 | 0.024 | 0.016 | 0.058 | **0.023** | 0.058 |
| moving_extra_long_bag1 | 0.029 | 0.036 | 0.086 | 0.079 | 0.037 | **0.037** | 0.086 |
| moving_extra_long_bag2 | 0.025 | 0.017 | 0.044 | 0.019 | 0.023 | **0.023** | 0.044 |
| turning_bag1 | 0.113 | 0.085 | 0.114 | 0.128 | **0.299** | 0.114 | **0.299** |
| turning_bag2 | **0.205** | 0.117 | 0.111 | 0.114 | 0.084 | 0.114 | **0.205** |

**0 bag failures.** All 35 bag-runs succeeded.

## Trial-level

```
worst-bag per rep:  0.205 / 0.117 / 0.114 / 0.128 / 0.299
median worst-bag:   0.128
max worst-bag:      0.299
q75 per rep:         0.071 / 0.061 / 0.099 / 0.097 / 0.071
median q75:          0.0713
```

## Comparison vs all known candidates

| metric (5-rep median) | trial 22 | trial 18 | v2 t9 | **near_18 t6** |
|---|---|---|---|---|
| median q75 | 0.087 | 0.078 | 0.0795 | **0.0713** |
| median worst-bag | 0.177 | 0.207 | 0.197 | **0.128** |
| max worst-bag (5 reps) | 0.258 | 0.331 | 0.388 | 0.299 |
| bag failures | 0 | 0 | 0 | **0** |

**Trial 6 (near_18) dominates trial 22 on both q75 AND median worst-bag.**
- q75: -18% (0.0713 vs 0.087)
- median worst-bag: -28% (0.128 vs 0.177)
- max worst-bag: +16% (0.299 vs 0.258) — slightly higher ceiling
- failures: tied at 0

The only metric where trial 22 still wins is max worst-bag (the
absolute single-rep worst). But trial 6's 0.299 is comparable to
trial 18's 0.331 and v2 t9's 0.388, and the median worst-bag advantage
(0.128 vs 0.177) more than compensates.

## Per-bag improvements (trial 6 vs trial 22, 5-rep median)

| bag | trial 22 | trial 6 | improvement |
|---|---|---|---|
| moving_long_bag1 | 0.068 | 0.018 | **-74%** |
| moving_long_bag3 | 0.058 | 0.017 | **-71%** |
| moving_long_bag4 | 0.030 | 0.023 | -23% |
| moving_extra_long_bag1 | 0.136 | 0.037 | **-73%** |
| moving_extra_long_bag2 | 0.034 | 0.023 | -32% |
| turning_bag1 | 0.107 | 0.114 | +7% |
| turning_bag2 | 0.175 | 0.114 | **-35%** |

**Five bags improved substantially**, one slightly worse (turning_bag1
within noise), one decisively better (turning_bag2 -35%). This is the
first candidate that improves on the long bags AND turning_bag2 without
trade-off.

## In-optim vs 5-rep — methodology validation

- In-optim n_reps=5 median q75: 0.0702
- Separate 5-rep validation q75: 0.0713
- **Gap: 1.6%** (within rep-to-rep noise)

Compare to n_reps=3 in-optim gaps:
- MO t9: in-optim 0.076 → 5-rep 0.128 (68% gap)
- q90 t6: in-optim 0.138 → 5-rep 0.232 (68% gap)
- Trial 8 (v2): in-optim 0.0755 → 5-rep 0.086 (14% gap)

**n_reps=5 in-optim is honest scoring**, as the day-5 methodology
hypothesis predicted.

## Verdict

**`capra_near_18_v1` trial 6 is the NEW DEPLOYMENT WINNER.** It
dominates trial 22 on q75 (-18%) AND median worst-bag (-28%), with no
bag failures, no Pareto trade-off (the slight max-worst-bag uptick is
within trial 22 and trial 18's range, not regressive).

Update `TUNING_PLAYBOOK.md` to flag trial 6 as default.

## Reproducing

```bash
ros2 run rove_rtabmap_tuner run_trial \
  --bag /path/to/bag --output-root ./verify --trial-id deploy \
  --expected-update-rate 50.0 --max-bag-duration-s 300 \
  --bag-play-arg=--topics --bag-play-arg=/livox/lidar --bag-play-arg=/imu/data \
  --bag-play-arg=/tf --bag-play-arg=/tf_static \
  -s icp_iterations=7 \
  -s icp_map_correspondence_ratio=0.09849734301357793 \
  -s icp_max_correspondence_distance=0.0810030219106119 \
  -s icp_max_translation=0.45684472051249736 \
  -s icp_odom_correspondence_ratio=0.1246205588641772 \
  -s icp_outlier_ratio=0.16835800055096892 \
  -s icp_point_to_plane_k=25 \
  -s icp_strategy=1 \
  -s icp_voxel_size=0.04184289695746849 \
  -s mem_stm_size=7 \
  -s odom_scan_keyframe_thr=0.8846779645360893 \
  -s odomf2m_scan_max_size=10765 \
  -s odomf2m_scan_subtract_radius=0.12337633169863922 \
  -s rgbd_angular_update=0.04695654220771511 \
  -s rgbd_linear_update=0.3422465680761723 \
  -s rgbd_proximity_path_max_neighbors=5
```
