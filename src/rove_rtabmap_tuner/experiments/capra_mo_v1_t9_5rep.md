# `capra_mo_v1` trial 9 — 5-rep validation (lucky in-optim, negative)

## Why this run exists

After resuming the MO NSGA-II study to 23 total trials, **trial 9** was the
sole Pareto-front point with in-optim values `q75=0.0759, max=0.1015`. If
real, that would have been a **2.5× improvement** on max-aggregation over
trial 22 (0.258) and matched trial 18 on q75 (0.078). 5-rep validation
on the 7-bag set tests whether the in-optim numbers hold.

## Setup

- Trial 9's params from `study_mo_v1/optuna.db`.
- 7-bag set, 5 reps, 2 parallel × 3 batches (CPU-aware).
- `ROS_DOMAIN_ID 90-94`.
- Wall: ~25 min.

## Per-bag results

| bag | r1 | r2 | r3 | r4 | r5 | median | max |
|---|---|---|---|---|---|---|---|
| moving_long_bag1 | 0.103 | 0.079 | 0.025 | 0.154 | 0.046 | 0.079 | 0.154 |
| moving_long_bag3 | 0.030 | 0.059 | 0.088 | 0.221 | 0.020 | 0.059 | 0.221 |
| moving_long_bag4 | 0.035 | 0.024 | 0.041 | 0.060 | 0.064 | 0.041 | 0.064 |
| moving_extra_long_bag1 | 0.088 | 0.051 | 0.127 | 0.034 | 0.127 | 0.088 | 0.127 |
| moving_extra_long_bag2 | 0.043 | 0.061 | 0.046 | 0.040 | 0.020 | 0.043 | 0.061 |
| turning_bag1 | **0.198** | **0.162** | **0.384** | 0.182 | **0.188** | **0.188** | **0.384** |
| turning_bag2 | 0.153 | 0.056 | 0.062 | **0.284** | 0.174 | 0.153 | 0.284 |

Bold = the worst bag for that rep. `turning_bag1` was worst in 4 of 5
reps.

## Trial-level

```
worst-bag per rep: 0.198 / 0.162 / 0.384 / 0.284 / 0.188
median worst-bag:  0.198
max worst-bag:     0.384
q75 per rep:        0.128 / 0.070 / 0.108 / 0.201 / 0.150
median q75:         0.128
```

## Comparison vs known winners

| metric (5-rep median) | trial 22 | trial 18 | MO t9 in-optim (3-rep) | **MO t9 5-rep** |
|---|---|---|---|---|
| median q75 | 0.087 | **0.078** | 0.076 | 0.128 |
| median worst-bag | **0.177** | 0.207 | n/a | 0.198 |
| max worst-bag (5 reps) | **0.258** | 0.331 | 0.102 | **0.384** |

**MO trial 9 was a lucky in-optim sample.** Its 5-rep numbers are:
- 68% worse q75 than in-optim (0.128 vs 0.076)
- **3.8× worse max** (0.384 vs 0.102)

The single-trial n_reps=3 in-optim score under-reported worst-case by
nearly 4×. Trial 22 is dominant on max-aggregation; trial 18 on q75.
Trial 9 dominates neither.

## Why this happened

`max_drift_per_path` is the noisiest metric in the toolkit because a
single bad rep on a single bag controls the value. With n_reps=3, you
need all 3 reps to coincidentally avoid a noisy bag (especially
`turning_bag1` here) to score low. With 5 reps, you usually hit at least
one bad rep, and the max climbs.

This pattern matches earlier findings:
- `#367` in-optim claimed max=0.16; 5-rep validation showed 0.354.
- `trial 13` (wide_v1) in-optim q75=0.135; 5-rep median 0.190.
- `trial 8` (max_v1) in-optim median max=0.253; never validated but
  in-optim was already not competitive.

NSGA-II with max-aggregation objective amplifies this noise problem — it
selects trials that *happened* to look good across 3 reps, not trials
that are robustly good.

## Recommended fix (out-of-scope for this block)

To make max-aggregation NSGA-II work:
- Use `n_reps_per_trial >= 5` in optim (slower but honest scores).
- Or substitute `q90` for `max` in the optim objective (less noisy but
  still tail-aware).
- Or implement a "stable_max" metric that's the median across n_reps'
  max values (already what `--n-reps-per-trial` gives, but at n=3 it's
  too few — need n≥5 to defeat the variance).

## Verdict

**Trial 22 remains deployment winner.** MO study did not find a
genuinely better Pareto candidate at this budget. Total wall on the
resumed MO study: ~4 hr (15 trials) plus 25 min validation. Net result:
confirmed the lesson that in-optim max-aggregation scores need many more
reps to be trustworthy, but no new deployable params.
