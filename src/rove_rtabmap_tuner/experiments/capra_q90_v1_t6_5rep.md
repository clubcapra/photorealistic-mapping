# `capra_q90_v1` trial 6 — 5-rep validation (lucky in-optim, negative)

## Idea

`max_drift_per_path` was too noisy as an optim objective (per the MO t9
negative). Try `q90_drift_per_path` — between q75 and max, less noisy
than max but still tail-aware. 8 trials in `near_22` space.

## Setup

- Study `capra_q90_v1`, near_22 space, q90 metric, 8 trials, n_jobs=2,
  n_reps=3.
- Wall: ~2 hr.
- Best: **trial 6** (DB), in-optim q90=0.138.

## 5-rep validation result

```
median worst-bag:  0.235  (trial 22: 0.177)
median q75:        0.181  (trial 22: 0.087)
median q90:        0.232  (in-optim was 0.138)
max worst-bag:     0.464  (trial 22: 0.258)
```

Per-bag medians:
- `moving_long_bag1`: 0.164 (trial 22: 0.068)
- `moving_long_bag3`: 0.016 (trial 22: 0.058)
- `moving_long_bag4`: 0.025 (trial 22: 0.030)
- `moving_extra_long_bag1`: 0.177 (trial 22: 0.136)
- `moving_extra_long_bag2`: 0.068 (trial 22: 0.034)
- `turning_bag1`: 0.124 (trial 22: 0.107)
- `turning_bag2`: 0.231 (trial 22: 0.175)

## Why it failed (same pattern as MO t9)

- In-optim q90 (0.138) was **1.7× lower than 5-rep q90 (0.232)** — the
  trial got lucky on a 3-rep sample.
- max worst-bag jumped from in-optim ~0.13 to 5-rep 0.464 — a 3.5×
  amplification.
- `turning_bag2` swings 0.15-0.46 across reps under these params.

Whether the optim objective is `max`, `q90`, or even `q75`, n_reps=3
in-optim sampling is **systematically biased toward lucky low scores**
for tail-aware metrics. The fix has to be more reps per trial, not a
different aggregation.

## Verdict

**No improvement.** Trial 22 (q75-driven, 5-rep-validated) and trial 18
(near_22 q75-driven, 5-rep-validated) remain the only deployable
candidates from the autonomous-tuning effort.

Recommendation for future tuning runs: use `--n-reps-per-trial 5` in
optim. Wall time is 1.67× longer but eliminates the
lucky-in-optim → bad-5-rep validation gap that has now killed three
candidates (MO t9, q90 t6, and the earlier `max_drift_per_path` study's
trial 8 which we didn't even bother to validate).
