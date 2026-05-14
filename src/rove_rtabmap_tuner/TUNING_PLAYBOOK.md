# Tuning playbook for rove_rtabmap_tuner

A working set of recipes and findings from the capra_full_v1 study (~700 trials).

## TL;DR — recommended deployment params

From the existing study, **trial #367** is the most robust set: drift/path ≤ 0.16 on every scored bag, with loop closures firing across the board. Better than the lowest-median trial because it has no catastrophic-on-one-bag tradeoff.

```bash
ros2 run rove_rtabmap_tuner run_trial \
  --bag /path/to/bag --output-root ./verify --trial-id deploy \
  --expected-update-rate 50.0 --max-bag-duration-s 300 \
  --bag-play-arg=--topics --bag-play-arg=/livox/lidar --bag-play-arg=/imu/data \
  --bag-play-arg=/tf --bag-play-arg=/tf_static \
  -s icp_voxel_size=0.054 \
  -s icp_max_correspondence_distance=0.0935 \
  -s icp_iterations=15 \
  -s icp_outlier_ratio=0.164 \
  -s icp_max_translation=0.345 \
  -s icp_point_to_plane_k=27 \
  -s icp_strategy=1 \
  -s icp_odom_correspondence_ratio=0.146 \
  -s icp_map_correspondence_ratio=0.119 \
  -s odom_scan_keyframe_thr=0.836 \
  -s odomf2m_scan_max_size=20541 \
  -s odomf2m_scan_subtract_radius=0.0554 \
  -s rgbd_linear_update=0.288 \
  -s rgbd_angular_update=0.077 \
  -s rgbd_proximity_path_max_neighbors=3 \
  -s mem_stm_size=12
```

Per-bag performance on this set:

| bag | drift_m | drift/path | loops |
|---|---|---|---|
| moving_extra_long_bag1 | 1.50 | 5.9% | 42 |
| moving_extra_long_bag2 | 0.28 | 4.0% | 10 |
| moving_long_bag1 | 0.09 | 4.6% | 9 |
| moving_long_bag2 | 0.03 | 0.2% | 28 |
| moving_long_bag3 | 0.33 | 3.3% | 10 |
| moving_long_bag4 | 0.12 | 0.6% | 20 |
| moving_short_bag1 | 0.14 | 5.4% | 2 |
| turning_bag1 | 0.17 | 16.0% | 1 |

## Key findings from the study

### 1. There is no universal champion

12 distinct trials win for 13 bags — different param regimes are best for different bags. This is *why* the optimizer plateaus. The current optimum (max=0.16) represents the best compromise, not a hard limit.

### 2. Blocker bags (worst-drift bag in most trials)

```
moving_short_bag2  → worst in 24% of trials
moving_long_bag1   → worst in 23%
moving_extra_long_bag3 → 11%
turning_bag2       → 7%
```

These bags' drift swings hugely with params (best ever 0.0005, worst 0.99). Removing them from the bag list lowers the max-aggregation floor immediately.

### 3. Every bag is solvable individually

Every bag has been driven to drift ≤ 0.012 by *some* trial. No bag is structurally impossible. The optimizer just can't find params that work on all of them simultaneously.

### 3a. Per-motion-pattern parameter clusters

Looking at the params that *win* on each bag type reveals a cluster pattern:

| motion type | voxel_size median | max_correspondence_distance median | outlier_ratio | mem_stm_size |
|---|---|---|---|---|
| short bags | 0.058 | 0.44 | 0.37 | 7.5 |
| long bags | **0.082** | **0.50** | 0.50 | 8.5 |
| extra-long bags | 0.057 | **0.16** | 0.51 | 12.0 |
| turning bags | 0.077 | 0.61 | **0.72** | 8.5 |

Observations:
- Long bags want **3× larger** correspondence distance than extra-long bags (0.50 vs 0.16). Different motion patterns demand different ICP search radii.
- Turning bags want **much stricter outlier rejection** (0.72 vs ~0.50 for moving bags). Pure rotation puts correspondences at varied distances from the axis — strict rejection drops bad pairs.
- Extra-long bags want **larger short-term memory** (12 vs 8.5). Probably because longer trajectories accumulate more revisit candidates that benefit from staying in WM.

This cluster pattern suggests **per-bag-type tuning** could improve over a single universal param set. Practical move: if you can classify bags by motion pattern at runtime (e.g., from `nav_msgs/Odometry` velocity profile), select params accordingly.

### 4. Aggregator choice changes which trial wins

| aggregator | best trial | what it rewards |
|---|---|---|
| median | #379 (0.022 median, **0.41 max**) | typical-bag performance — hides outliers |
| q75 | #348 (0.042 q75) | typical + worst-quartile awareness |
| q90 | #367 (0.089 q90) | strong outlier penalty without single-bag domination |
| max | #360 (0.117 max, only 5 bags) | strict robustness — but fewer-bag trials win by luck |

**For loss-of-tracking criterion: use q75 or q90.** Max is too sensitive to a single outlier bag. Median hides catastrophes.

### 4. RTAB-Map appears to have measurable non-determinism

Empirical observation from re-running trial #367's params: drift on
`moving_long_bag1` was **0.046** in the original trial, **0.136** when
re-run later with the same params. ~3× different score from the same input.

This means:
- The optimizer's objective is noisy. A trial that scores 0.05 might score
  0.15 if re-run. TPE assumes a deterministic objective; with significant
  noise, it converges slower and treats "lucky" trials as best.
- Many of the "best" trials in capra_full_v1 may be partially lucky.
  Trial #367's robust profile across 8 bags is stronger evidence than a
  single-bag low-drift trial, since it's less likely all 8 got lucky at once.
- **Practical mitigation**: run the apparent best trial 3-5 times on the
  same bag set to estimate the noise. If the variance is large, you may
  need to either (a) average objective over repeated runs (~3× more expensive)
  or (b) accept that the optimizer is converging in noise.

Suspected causes: ROS 2 messaging order in concurrent subscribers, thread
scheduling in icp_odometry's PCL ops, ICP iteration count from inlier
floating-point ties. Hard to make fully deterministic without rebuilding
RTAB-Map with deterministic flags.

## Recipes

### Resume an existing study

```bash
ros2 run rove_rtabmap_tuner optimize \
  --bag <bag1> [--bag <bag2> ...] \
  --output-root /path/to/study \
  --study-name <existing_study_name> \
  --metric q90_drift_per_path \   # or whichever aggregation
  --n-trials 100 --n-jobs 6 \
  --max-bag-duration-s 300
```

### Defeat non-determinism with `--n-reps-per-trial`

If you've confirmed (per `experiments/nondeterminism_3rep.md`) that the bag
set is noisy enough that single-run scores are unreliable, run each Optuna
trial multiple times and take the median:

```bash
ros2 run rove_rtabmap_tuner optimize \
  --bag <bag1> [--bag <bag2> ...] \
  --output-root /path/to/study \
  --study-name <name> \
  --metric q75_drift_per_path \
  --n-trials 60 --n-jobs 4 --n-reps-per-trial 3
```

Wall time = N×N×(bag-time) for the run; e.g. 60 trials × 3 reps × 9 bags at
~80s/bag with 4 parallel = ~9 hours. Significant cost but much more reliable
than the same total compute spread across more single-run trials.

`load_if_exists=True` is implicit. Stale `RUNNING` trials from previous kills auto-fail at startup. Incomplete trial directories get cleaned up.

### Force-kill safely

Just `Ctrl-C` or `kill <pid>`. The signal handler:
1. SIGINTs every subprocess group it spawned (rtabmap, icp_odometry, ros2 bag play, etc.).
2. Escalates to SIGTERM then SIGKILL if a process doesn't exit cleanly.
3. Exits with conventional signal code.

Next startup auto-self-heals: orphan `RUNNING` trials → `FAIL`, incomplete trial dirs deleted.

### Find the most robust trial in a study

```bash
ros2 run rove_rtabmap_tuner analyze_per_bag /path/to/study
```

Top-5 by max, q75, q90, median against the same data — pick whichever matches your robustness preference.

### Rerank an existing multi-objective study by a different metric

```bash
ros2 run rove_rtabmap_tuner rank_trials /path/to/study \
  --metric mean_icp_ratio --top 10
```

## When to start a fresh study vs resume

| situation | move |
|---|---|
| Same metric, more trials | resume (`--study-name X` same as before) |
| New metric with different aggregation | new study (Optuna won't mix directions) |
| Changed `SEARCH_SPACE` bounds (narrowed) | new study or accept stale priors with out-of-bound values |
| Changed `SEARCH_SPACE` (added a param) | new study (Optuna handles missing-param priors poorly) |
| Different bag set | new study (per-bag user_attrs are positional, comparison invalid) |

## Newly-overridable params (not auto-tuned, opt-in via `--set`)

These have placeholders in the template + defaults in `template_renderer.py` but are **not** in `SEARCH_SPACE`, so the optimizer won't auto-explore them on existing studies. Test manually first, then graduate to SEARCH_SPACE in a fresh study.

- `icp_force_3dof` (default `false`): restrict ICP to (x, y, yaw). For ground robots, eliminates Z/roll/pitch noise.
- `icp_force_4dof` (default `false`): restrict to (x, y, z, yaw). Keep height but lock roll/pitch — useful on inclines.

Existing-already-overridable but-not-auto-tuned:
- `odomf2m_bundle_adjustment` (`'true'`/`'false'`): enables sliding-window BA in ICP odometry. ~2-3× CPU cost, may improve accuracy.
- `reg_strategy` (`'0'`/`'1'`/`'2'`): registration strategy at map level. We keep `'1'` (ICP) for lidar-only setups.

Test pattern: render with the candidate enabled, run a single trial against a couple bags, compare drift vs the trial-#367 baseline before committing to a tuning study with the param added to SEARCH_SPACE.

```bash
ros2 run rove_rtabmap_tuner run_trial \
  --bag /home/iliana/bags/moving_long_bag1 \
  --bag /home/iliana/bags/moving_extra_long_bag1 \
  --output-root /tmp/force3dof_smoke --trial-id force_3dof \
  -s icp_force_3dof=true [other -s from #367's params] \
  --expected-update-rate 50.0 --max-bag-duration-s 300 \
  --bag-play-arg=--topics --bag-play-arg=/livox/lidar --bag-play-arg=/imu/data \
  --bag-play-arg=/tf --bag-play-arg=/tf_static
```

## Open candidate improvements (still not implemented)

1. **Custom `RGBD/OptimizeStrategy` (graph backend choice)**: TORO vs g2o vs GTSAM vs Ceres. Categorical, 4 choices.
2. **GPS-based ATE metric**: The current "drove a loop, returned to start" framing is structurally limited (no ground truth at intermediate poses). A GPS-tagged ATE per-pose unlocks proper SLAM-quality scoring.
3. **Per-bag tuning** (ensemble): rather than a single param set, learn per-bag classifiers that select params from a Pareto front. Significantly more complex but would push past the "no universal champion" wall.
