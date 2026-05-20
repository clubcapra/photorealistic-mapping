# Tuning playbook for rove_rtabmap_tuner

A working set of recipes and findings from the capra_full_v1 study (~700 trials).

## TL;DR — recommended deployment params

**Updated 2026-05-15 — overnight `capra_focused_v3` + day-2 follow-up experiments.**

| candidate | median worst-bag (5 rep) | median q75 (5 rep) | max worst-bag (5 rep) | source / status |
|---|---|---|---|---|
| **`capra_focused_v3` trial 22** ← **DEPLOYMENT WINNER** (default — robust) | **0.177** | 0.087 | **0.258** | `experiments/trial_22_5rep.md` |
| **`capra_near_22_v1` trial 18** (q75 Pareto alt) | 0.207 | **0.078** | 0.331 | `experiments/near_22_t18_5rep.md` |
| **`capra_near_22_v2` trial 9** (q75 Pareto alt v2) | 0.197 | 0.0795 | 0.388 | `experiments/capra_near_22_v2_t9_5rep.md` (new — n_reps=5 optim) |
| `capra_near_22_v2` trial 8 | 0.215 (1 FAIL) | 0.086 | 1.0 (FAIL) | `experiments/capra_near_22_v2_t8_5rep.md` (n_reps=5 in-optim winner; failed bag3 in 1/5 reps) |
| `capra_focused_v3` trial 10 | 0.180 | 0.130 | 0.289 | `experiments/capra_focused_v3_winner_5rep.md` |
| `study_full` trial 349 (long-bag specialist v2) | 0.233 | 0.099 | 0.414 | `experiments/study_full_trial_349_5rep.md` (5-bag median max=0.056, on-par with original) |
| `#367` (historical baseline) | 0.221 | 0.148 | 0.354 | `experiments/baseline_367_7bag_5rep.md` |
| `capra_wide_v1` trial 13 (wide-search) | 0.244 | 0.190 | 0.382 | `experiments/capra_wide_v1_t13_5rep.md` (negative) |
| `capra_turning_v1` t4 (turning-only optim) | 0.257 | 0.096 | 0.432 | `experiments/capra_turning_v1_t4_5rep.md` (negative) |
| `capra_max_v1` trial 8 (max-metric optim) | not validated | n/a | n/a | in-optim 3-rep median = 0.253, run-time evidence enough — not competitive |

**Pareto note (trial 18 vs trial 22)**: trial 18 wins decisively on long
bags (median drift halved on `moving_long_bag1`, `moving_long_bag3`, and
`moving_extra_long_bag1` — see writeup). Trial 22 is more robust on
turning bags. Use trial 22 by default, trial 18 if the deployment workload
is predominantly forward motion. Both came from `near_22` neighborhood,
just different points on the q75↔max Pareto front.

Day-2 follow-up experiments (all negative — trial 22 unchanged as winner):
- **Wide-search**: 17-trial wide-space study. Best wide trial 13 has worst-bag 0.244, 38% worse than trial 22. `near_367` basin is genuinely good.
- **Turning-only optim**: 7-trial study with `turning_bag1+2` only. Winner over-fit (median worst-bag 0.257 on full set).
- **`icp_force_3dof=true`** smoke: turning bags 50-80% worse. See `experiments/icp_force_dof_smokes.md`.
- **`icp_force_4dof=true`** smoke: turning bags 40-250% worse. Same writeup.
- **`study_full` trial 349 cross-validated**: long-bag specialist (excellent on 5 long bags) but turning bags drift 0.04-0.41.

Day-4 follow-up experiments (all negative — confirming a methodology issue):
- **MO NSGA-II (q75 + max), 23 trials**: best Pareto trial (#9) had in-optim q75=0.076 max=0.102 but 5-rep showed q75=0.128 max=0.384 — **lucky in-optim sample**. See `experiments/capra_mo_v1_t9_5rep.md`.
- **q90 single-obj, 8 trials**: best trial 6 in-optim q90=0.138 → 5-rep q90=0.232 — same pattern. See `experiments/capra_q90_v1_t6_5rep.md`.

Day-5 (n_reps=5 in-optim experiment):
- **capra_near_22_v2**: 20-trial study with `n_reps=5` per trial (vs n_reps=3 previously). Two winners 5-rep-validated:
  - **trial 8 (in-optim q75=0.0755)**: 5-rep q75=0.086 (within 14% of in-optim, much tighter than n_reps=3 gaps) — but failed `moving_long_bag3` in 1 of 5 validation reps. n_reps=5 gives more honest in-optim scores but doesn't catch rare failure modes.
  - **trial 9 (in-optim q75=0.0920)**: 5-rep q75=0.0795 — tied with trial 18, 0 failures. Pareto-equivalent.
- **The q75 floor in the near_22 basin is ~0.08**, repeatable across 3 different runs with different priors. The worst-bag floor (~0.18) is set by RTAB-Map's non-determinism on `turning_bag2` and `moving_long_bag1`.

**METHODOLOGY LESSON (from 3 candidates killed by this same issue)**: n_reps=3
in-optim scoring is systematically **biased low** for tail-aware aggregators
(`max`, `q90`, even `q75` to a smaller degree). The optimizer picks
candidates that *happen* to look good across 3 reps; 5-rep validation
reveals the true distribution and the candidate snaps back.

**Recommendation for future tuning**: use `--n-reps-per-trial 5` in optim
for any tail-aware metric. Wall time is 1.67× longer but the
lucky-sample → bad-validation gap disappears. Or stick with q75 metric
(less affected — see trial 18's 5-rep q75=0.078 matching its in-optim
0.098 within reasonable noise).

**Headline:** Trial 22 is the new deployment winner. Two near-tied q75 optima
came out of the `capra_focused_v3` 24-trial study (10 → 0.0975, 22 → 0.0983
on in-optim n_reps=3). 5-rep validations on the same 7-bag set showed:

- Trial 22's 5-rep q75 is **0.087** — *lower than its in-optim score*. Trial 10's 5-rep q75 is 0.130 — *higher than its in-optim score* (i.e., trial 10 was a lucky in-optim sample, trial 22 was honest).
- Trial 22 has a tighter worst-case ceiling (max 0.258 vs trial 10's 0.289 vs #367's 0.354).
- Median worst-bag is essentially tied between trial 22 and trial 10 (0.177 vs 0.180).

Use trial 22's params for deployment (block below). Trial 10 and #367 are
both retained in the doc as historical reference points and Pareto-comparable
candidates.

The `capra_max_v1` study (optimizing max-aggregation directly, 10 trials)
failed to find an improvement over #367 in this search space; max metric is
too noisy without more reps per trial.

### Trial 22 deployment block (NEW recommendation)

```bash
ros2 run rove_rtabmap_tuner run_trial \
  --bag /path/to/bag --output-root ./verify --trial-id deploy \
  --expected-update-rate 50.0 --max-bag-duration-s 300 \
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
  -s rgbd_proximity_path_max_neighbors=4
```

Notable departures from trial 10: ~1.5× larger voxel (0.044), tighter ICP
correspondence radius (0.094), stricter outlier rejection (0.13), and a much
larger `rgbd_linear_update` (0.42 — fewer keyframes per metre). Trial 22
keyframes less aggressively but matches ICP more precisely.

> See also `experiments/moving_extra_long_bag4_failure_cause.md` —
> bag4's "no trajectory" failure mode is structural (missing
> `/tf`/`/tf_static` topics + 232 s playback > the 180 s cap). Exclusion
> from eval is correct, not a tuning failure.

### Trial 10 deployment block (PRIOR recommendation — superseded by trial 22 above)

```bash
ros2 run rove_rtabmap_tuner run_trial \
  --bag /path/to/bag --output-root ./verify --trial-id deploy \
  --expected-update-rate 50.0 --max-bag-duration-s 300 \
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
  -s rgbd_proximity_path_max_neighbors=2
```

Notable departure from #367: `icp_voxel_size` ≈ 0.033 (~60% of #367's 0.054);
`icp_max_correspondence_distance` ≈ 0.144 (~1.5× #367's 0.0935);
`icp_outlier_ratio` ≈ 0.27 (vs 0.16); `odomf2m_scan_max_size` ≈ 11831 (vs
20541, much smaller scan map). Trial 10 prefers a finer voxel grid and a
larger correspondence search distance — a slightly different ICP operating
point that turns out to be better on this bag set.

---

### Historical: `#367` (the old recommended baseline)

From `capra_full_v1` (700-trial study), **trial #367** was the previous
recommended baseline. Earlier `experiments/trial_367_validation_3rep.md`
reported **3-rep** median worst-bag of 0.27 (9 bags) / 0.137 (8 bags). The
5-rep validation in `baseline_367_7bag_5rep.md` gives a tighter and slightly
worse number: median worst-bag = 0.221 on 7 bags. The 0.137 was lucky.

Long bags reliably ≤0.05 drift under #367; the per-rep variance comes mostly
from `moving_long_bag1`, `turning_bag1`, and `turning_bag2`.

#367's full param block (still useful as a comparison point):

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
