# `capra_near_22_v1` — narrow search around trial 22 (partial / negative)

## Idea

Add a `near_22` search space (anchored around `capra_focused_v3` trial 22's
params, ±30-40% ranges) and refine TPE there. Trial 22 is the deployment
winner from the much broader `near_367` search.

## Code change

Added `SEARCH_SPACE_NEAR_22` to `optimizer.py`, exposed via
`--search-space near_22` CLI flag. Anchored on trial 22's 16 values.

## Run

- Smoke: 2 trials, n_reps=2, n_jobs=2 — verified the space loads and
  produces sensible scores.
- Scale: 4 more trials at n_reps=3, n_jobs=2.
- **Aborted at 4 COMPLETE + 2 RUNNING** to respect the autonomous block
  deadline (user needed run wrapped by 18:00).

## Results

```
trial 1: q75 = 0.460  (random init)
trial 2: q75 = 0.165  (random init)
trial 3: q75 = 0.252  (TPE-guided)
trial 4: q75 = 0.183  (TPE-guided)
```

**No trial beat trial 22's 5-rep q75 of 0.087** in this small budget.
Both random-init and TPE-guided values stayed at 0.16-0.46.

## Why this doesn't prove `near_22` is bad

Only 2 trials of TPE-guided exploration (3 and 4). TPE typically needs
8-15 trials to converge in a 16-dim space, even narrow. The 2 we saw were
moving in the right direction (0.252 → 0.183) but not nearly there.

To rule out `near_22` properly: run 15+ trials at n_reps=3 (~3.5 hr at
n_jobs=2). Out of scope for today's block.

## Verdict

The code change for `SEARCH_SPACE_NEAR_22` is committed; the negative
result is partial — more trials would tell us conclusively whether
`near_22` can refine on `near_367`'s findings. **Trial 22 remains the
deployment winner**.

Recommended follow-up for a future block:
```bash
# Resume capra_near_22_v1 with more trials
ros2 run rove_rtabmap_tuner optimize \
  --bag /home/iliana/bags/moving_long_bag1 \
  --bag /home/iliana/bags/moving_long_bag3 \
  --bag /home/iliana/bags/moving_long_bag4 \
  --bag /home/iliana/bags/moving_extra_long_bag1 \
  --bag /home/iliana/bags/moving_extra_long_bag2 \
  --bag /home/iliana/bags/turning_bag1 \
  --bag /home/iliana/bags/turning_bag2 \
  --output-root /home/iliana/prog/study_near_22_v1 \
  --study-name capra_near_22_v1 \
  --metric q75_drift_per_path \
  --search-space near_22 \
  --n-trials 15 --n-jobs 2 --n-reps-per-trial 3 \
  --seed 224 \
  --max-bag-duration-s 180 --expected-update-rate 50.0 \
  --bag-play-arg=--topics --bag-play-arg=/livox/lidar --bag-play-arg=/imu/data \
  --bag-play-arg=/tf --bag-play-arg=/tf_static
```
