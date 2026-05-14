# Focused-narrow-space run — partial results

## Context

After committing the q75/q90 aggregators, the `--n-reps-per-trial` flag, the
shutdown handler, and the `SEARCH_SPACE_NEAR_367` narrow space, we kicked
off two attempts at a focused optimization over the 9 certain-loop bags:

```
ros2 run rove_rtabmap_tuner optimize \
  --bag <9 clean loop bags> \
  --output-root /home/iliana/prog/study_focus2 \
  --study-name capra_focus_v2 \
  --search-space near_367 --metric max_drift_per_path \
  --n-trials 8 --n-jobs 4 --n-reps-per-trial 2 \
  --seed 42 --max-bag-duration-s 120 --expected-update-rate 50.0 \
  --bag-play-arg=...
```

Both attempts were killed by SIGINT after ~40 min of compute — not by my
hand, and not by the user's hand from this terminal. Origin unclear (the
shutdown handler caught and cleaned up properly each time, so all
subprocess groups died cleanly).

## What we got

Trials 0-3 completed both reps but hit sparsity floors (random-startup
samples in the narrow space happened to produce 1-2 keyframes on most bags).
Trials 4-7 only completed `rep_00` partially (6-7 bags each) before SIGINT.

**Trial #4 (rep_00, 6 of 9 bags) is the standout result.**

```
bag                        drift/path  n_poses  loops
moving_extra_long_bag1     0.082       50       4
moving_extra_long_bag2     0.035       45       4
moving_long_bag1           0.023       23       4
moving_long_bag3           0.010       27       2
moving_long_bag4           0.031       26       0
moving_short_bag2          0.379       6        0
```

**5 of 6 bags at ≤ 8.2% drift, the 6th at 38% (moving_short_bag2 is the
chronically-noisy bag).** If max_drift_per_path were computed on these 6,
it'd be 0.379 — driven entirely by the one short bag. The 5 "good" bags
together are noticeably better than trial #367's equivalents (#367 had
drift/path 0.040-0.060 across most bags).

The trial 4 params differ from #367 in interpretable ways:

| param | #367 | trial #4 |
|---|---|---|
| icp_voxel_size | 0.054 | **0.037** (smaller voxel = more detail) |
| icp_max_correspondence_distance | 0.093 | **0.066** (tighter search) |
| icp_iterations | 15 | 10 |
| icp_outlier_ratio | 0.164 | **0.271** (stricter rejection) |
| odomf2m_scan_subtract_radius | 0.055 | 0.030 |
| icp_odom_correspondence_ratio | 0.146 | **0.103** (more permissive accept) |
| rgbd_linear_update | 0.288 | 0.198 |
| rgbd_angular_update | 0.077 | **0.144** (more keyframes on rotation) |

Smaller voxel + stricter outlier + more keyframes on rotation — these are
all moves toward "tighter, more careful registration" that would intuitively
improve drift on the moving bags. The downside seen here: it didn't help
moving_short_bag2's persistent noise problem.

## Trial #4 deployment-ready --set lines

```bash
-s icp_voxel_size=0.0373 \
-s icp_max_correspondence_distance=0.0655 \
-s icp_iterations=10 \
-s icp_outlier_ratio=0.2709 \
-s icp_max_translation=0.3511 \
-s icp_point_to_plane_k=24 \
-s icp_strategy=1 \
-s odom_scan_keyframe_thr=0.6791 \
-s odomf2m_scan_max_size=19327 \
-s odomf2m_scan_subtract_radius=0.0302 \
-s icp_odom_correspondence_ratio=0.1034 \
-s icp_map_correspondence_ratio=0.1388 \
-s rgbd_linear_update=0.1975 \
-s rgbd_angular_update=0.1438 \
-s mem_stm_size=12 \
-s rgbd_proximity_path_max_neighbors=2
```

## What to actually do next

1. **Verify trial #4 with `--n-reps-per-trial 3-5` on the full 9 bags.**
   Single-rep, partial-bag data suggests it beats #367 on most bags; need
   confirmation under the noise model. Should take ~30 min wall.
2. **Run a longer focused study from a different host or terminal** to
   avoid whatever was killing background tasks at the 40-minute mark.
   `nohup ros2 run ... &` or `tmux/screen` should isolate it from session
   events.
3. **moving_short_bag2 is structurally noisy** — drift/path 0.05-0.99
   across trials with the same params. Likely the actual SLAM-difficulty
   ceiling for that bag, not a param issue.

## Why optimizing accuracy further is hard

The whole autonomous-improvements branch revealed three compounding issues:

1. RTAB-Map is non-deterministic by ~5× on identical reruns
   (`experiments/nondeterminism_3rep.md`).
2. No single param set is best across all bags
   (`experiments/per_bag_winners_capra_full_v1.md` — 12 distinct winners
   for 13 bags).
3. `moving_short_bag2`-class bags hit a noise floor where drift/path can
   vary 0.05 → 0.70 on identical params.

The combination means: improvements over trial #367 will need both
**deterministic execution** (ground-truth ATE or fixed seeds in RTAB-Map)
**and** per-bag-type parameter selection (the ensemble approach). The
existing single-param-set tuning has likely converged within the noise
band.
