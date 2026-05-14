# Non-determinism: 3 reps of trial #367's params on `moving_long_bag1`

## Setup

Identical conditions for all 3 reps:
- bag: `/home/iliana/bags/moving_long_bag1` (75s playback)
- params: trial #367 (see `../TUNING_PLAYBOOK.md`)
- `icp_force_3dof=false`
- single bag per trial, sequential runs (no parallelism)
- warmup 8s, drain 3s, max-bag-duration 120s
- ROS_DOMAIN_ID set automatically (default 0)
- wall-time total: 4.4 minutes

## Results

| rep | drift_m | drift_per_path | n_poses | path_length | loop_closures |
|---:|--------:|---------------:|--------:|------------:|--------------:|
| 1   |   0.60  | 0.134          |    27   |    4.5 m    |    20         |
| 2   |   1.63  | 0.348          |    28   |    4.7 m    |    10         |
| 3   |   2.88  | 0.241          |    30   |   12.0 m    |     4         |

## Spread

- **drift_m: 0.60 → 2.88 — 4.8× variance.**
- drift_per_path: 0.13 → 0.35 — 2.6× variance.
- path_length: 4.5 → 12.0 m — 2.7× variance.
- loop_closures: 4 → 20 — 5× variance.

These aren't small noise effects — they're the range that the entire
optimization study was trying to push improvements *into*. The historical
"best max_drift_per_path = 0.0552" could plausibly have been a 0.15 trial
that got lucky on its single run.

## Implications

1. **The optimizer was tuning in noise** for at least the last several
   hundred trials. The "plateau" since trial #186 / #324 isn't necessarily
   "we found the optimum" — it could be "noise floor is around 0.05-0.15
   and we keep getting lucky/unlucky picks within that band."

2. **Single-run trial scores are unreliable.** Comparing two trials by a
   single drift number gives noisy ranking; many "best" trials may simply
   be the lucky tail of the noise distribution.

3. **Trial #367's robust-across-8-bags profile is stronger evidence** than
   a single-bag low-drift trial. The probability that all 8 bags got lucky
   simultaneously is much lower than one bag getting lucky.

## What to do

Option A — accept the noise, run more trials. With ~30% noise on each
score, TPE can still distinguish "drift ~ 0.05 region" from "drift ~ 0.5
region" given enough samples. It just can't reliably pick the best 0.05
from the best 0.06. You already have the right ballpark.

Option B — average over reps. Modify the objective to run each param set
3-5 times per bag and average the drift. Pros: noise reduced by sqrt(N).
Cons: 3-5× more wall time per trial.

Option C — fix the non-determinism source. Suspected culprits, in order of
suspicion:
- ICP iteration tie-breaking on identical-score correspondences (PCL or
  libpointmatcher; mostly inside the C++).
- Subscriber ordering on `/livox/lidar` ↔ `/imu/data` topic sync (ROS 2
  DDS doesn't guarantee receive ordering).
- Thread scheduling in icp_odometry's parallel scan-cloud filtering.
This requires rebuilding RTAB-Map with deterministic flags (or fixed seeds
in the relevant code paths). Probably not tractable for the team.

For practical deployment, **Option A + Option B's variance estimate** is
the most realistic path:

```bash
# Run the candidate deployment params on every bag 5x and look at variance:
for rep in 1 2 3 4 5; do
  ros2 run rove_rtabmap_tuner run_trial \
    --bag /path/to/bag --output-root /tmp/variance_check \
    --trial-id rep_$rep \
    [trial #367's full -s lines]
done
# Then median + IQR per bag → that's your real confidence interval.
```

## Reproduction

```bash
for rep in 1 2 3; do
  ros2 run rove_rtabmap_tuner run_trial \
    --bag /home/iliana/bags/moving_long_bag1 \
    --output-root /tmp/nondet_test --trial-id rep_$rep \
    --expected-update-rate 50.0 --max-bag-duration-s 120 \
    -s icp_voxel_size=0.054 -s icp_max_correspondence_distance=0.0935 \
    -s icp_iterations=15 -s icp_outlier_ratio=0.164 -s icp_max_translation=0.345 \
    -s icp_point_to_plane_k=27 -s icp_strategy=1 \
    -s icp_odom_correspondence_ratio=0.146 -s icp_map_correspondence_ratio=0.119 \
    -s odom_scan_keyframe_thr=0.836 -s odomf2m_scan_max_size=20541 \
    -s odomf2m_scan_subtract_radius=0.0554 -s rgbd_linear_update=0.288 \
    -s rgbd_angular_update=0.077 -s rgbd_proximity_path_max_neighbors=3 \
    -s mem_stm_size=12 -s icp_force_3dof=false \
    --bag-play-arg=--topics --bag-play-arg=/livox/lidar --bag-play-arg=/imu/data \
    --bag-play-arg=/tf --bag-play-arg=/tf_static
done
```
