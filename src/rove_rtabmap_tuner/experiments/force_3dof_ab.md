# A/B experiment: `Icp/Force3DoF` against trial #367's params

## Setup

Same param overrides as trial #367 (see `../TUNING_PLAYBOOK.md`), with the
only varying knob being `icp_force_3dof`.

Bags: `moving_long_bag1` (75s), `moving_short_bag2` (39s).
(A third bag was attempted but had been renamed `uncertain_*` earlier —
only 2 valid bags contributed data.)

Each trial run sequentially with `run_trial`, RTAB-Map fresh per bag,
`--max-bag-duration-s 180`. Wall-time: 4.6 min total.

## Results

| bag                | force_3dof | drift_m | path_length (inferred) | drift/path | n_poses | loops |
|--------------------|-----------|--------:|----------------------:|----------:|-------:|------:|
| moving_long_bag1   | **false** |    1.80 |                  13.2 m |    0.136  |     31 |     4 |
| moving_long_bag1   | **true**  |    1.03 |                   4.5 m |    0.229  |     29 | **16** |
| moving_short_bag2  | **false** |    0.51 |                   2.7 m |    0.192  |     13 |     2 |
| moving_short_bag2  | **true**  |    0.99 |                   2.2 m |    0.456  |     10 |     0 |

## Interpretation

**Force3DoF is bag-dependent — helps long bag, hurts short bag.**

- `moving_long_bag1`: drift_m dropped 43% (1.80 → 1.03), loop closures
  jumped 4×. *But* drift_per_path went up because the constrained motion
  model produced a shorter trajectory estimate (4.5 m vs 13.2 m). The raw
  drift improvement is real; the relative ratio is misleading because the
  denominator changed.
- `moving_short_bag2`: drift_m almost doubled (0.51 → 0.99), zero loop
  closures vs 2 in baseline. Clear regression.

The short bag likely has real Z motion (lidar pitch as the robot drives
over uneven terrain) that the 3DoF constraint can't represent, so ICP has
to fit the residual into x/y/yaw, producing distortion. The longer bag's
motion is more dominantly planar, so the constraint helps suppress noise.

## Recommendation

**Don't add `icp_force_3dof` to SEARCH_SPACE.** Treat it as a per-bag-context
override.

If the deployment scenario is one where the robot reliably stays on a flat
surface, `--set icp_force_3dof=true` may be worth it for the loop-closure
boost. Otherwise leave it off.

## Caveat: RTAB-Map appears non-deterministic

The same params on the same bag (trial #367's `moving_long_bag1`, drift
0.046) re-ran with drift 0.136 — ~3× different. The A/B results above are
single-run; some of the gap may be noise rather than the force_3dof effect.
A more rigorous experiment would run each cell 3-5 times.

## Reproducing

```bash
ros2 run rove_rtabmap_tuner run_trial \
  --bag /home/iliana/bags/moving_long_bag1 \
  --bag /home/iliana/bags/moving_short_bag2 \
  --output-root /tmp/force3dof_smoke \
  --trial-id with_force3dof_true \
  --expected-update-rate 50.0 --max-bag-duration-s 180 \
  -s icp_voxel_size=0.054 -s icp_max_correspondence_distance=0.0935 \
  -s icp_iterations=15 -s icp_outlier_ratio=0.164 -s icp_max_translation=0.345 \
  -s icp_point_to_plane_k=27 -s icp_strategy=1 \
  -s icp_odom_correspondence_ratio=0.146 -s icp_map_correspondence_ratio=0.119 \
  -s odom_scan_keyframe_thr=0.836 -s odomf2m_scan_max_size=20541 \
  -s odomf2m_scan_subtract_radius=0.0554 -s rgbd_linear_update=0.288 \
  -s rgbd_angular_update=0.077 -s rgbd_proximity_path_max_neighbors=3 \
  -s mem_stm_size=12 -s icp_force_3dof=true \
  --bag-play-arg=--topics --bag-play-arg=/livox/lidar --bag-play-arg=/imu/data \
  --bag-play-arg=/tf --bag-play-arg=/tf_static
```

Swap `icp_force_3dof=true` for `=false` to get the baseline cell.
