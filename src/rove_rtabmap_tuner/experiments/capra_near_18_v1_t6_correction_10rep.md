# `capra_near_18_v1` trial 6 — corrected 10-rep analysis (NOT a dominant winner)

## Why this exists

The first 5-rep validation of trial 6 (see `capra_near_18_v1_t6_5rep.md`)
reported it as dominating trial 22 on q75 (-18%) and median worst-bag
(-28%) with 0 failures. That conclusion was **premature**.

A second independent 5-rep validation (different ROS_DOMAIN_IDs 30-34,
otherwise identical setup) found a `moving_long_bag3` FAILURE in rep 1
that the first validation missed entirely. Combining both validations
gives a 10-rep characterization.

## 10-rep combined results (val1 IDs 60-64 + val2 IDs 30-34)

| bag | n_valid | median | max | fails |
|---|---|---|---|---|
| moving_long_bag1 | 10 | 0.0148 | 0.0580 | 0 |
| moving_long_bag3 | 9 | 0.0234 | 0.1943 | **1** |
| moving_long_bag4 | 10 | 0.0221 | 0.1010 | 0 |
| moving_extra_long_bag1 | 10 | 0.0448 | 0.1130 | 0 |
| moving_extra_long_bag2 | 10 | 0.0191 | 0.0437 | 0 |
| turning_bag1 | 10 | 0.1138 | 0.2988 | 0 |
| turning_bag2 | 10 | 0.1154 | 0.2049 | 0 |

**Bag failure rate: 1/70 = 1.4%** (1 RTAB-Map trajectory failure on
moving_long_bag3 across 10 reps).

## Trial-level (10 reps)

```
worst-bag per rep (FAIL=1.0): [0.205, 0.117, 0.114, 0.128, 0.299,
                               1.000, 0.194, 0.190, 0.103, 0.155]
median worst-bag (with FAIL penalty): 0.173
median q75 (excluding fail-rep):      0.088
max worst-bag:                        1.000 (rep 6 = val2 rep 1 FAILED)
```

## Honest comparison vs trial 22

| metric | trial 22 (5-rep) | trial 6 (10-rep corrected) | verdict |
|---|---|---|---|
| median worst-bag | **0.177** | 0.173 | TIED |
| median q75 | 0.087 | **0.088** | TIED |
| max worst-bag | **0.258** | 1.000 (1 FAIL) | trial 22 wins |
| bag failure rate | **0%** | 1.4% | trial 22 wins |

**Trial 6 is NOT dominant over trial 22.** They are essentially tied on
median q75 and median worst-bag. Trial 6 has the failure-rate concern
that trial 22 doesn't.

## What val1 alone missed

Val1's 5 reps happened to never trigger the bag3 failure mode. The
in-optim 5 reps also missed it. Only val2 (a separate 5-rep run with
different DDS domain IDs and different system load) caught it.

This is a methodology warning: **5-rep validation is insufficient to
characterize rare failure modes**. With ~10% per-rep failure
probability, a 5-rep run has ~60% chance of missing the failure
entirely. Need 10-15 reps for honest characterization of trial-level
reliability.

## Methodology lessons (updating day-5 conclusions)

1. **n_reps=5 in-optim is more honest than n_reps=3** for the in-optim
   *median q75* metric. Confirmed: trial 6 in-optim q75 was 0.0702 and
   its 10-rep q75 is 0.088 — a 25% gap, much smaller than the n_reps=3
   gaps (50-70%).

2. **But n_reps in optim cannot detect rare structural failures**. The
   in-optim trial 6 ran 5 reps and never hit the bag3 failure; val1 ran
   5 reps and never hit it either. Only val2 caught it.

3. **Future deployments should require 10-rep validation**, not 5-rep.
   The 2× wall cost is worth the failure-rate confidence.

## Revised verdict

**Trial 22 remains the deployment default.** Trial 6 is a Pareto-comparable
alternative with similar q75/worst-bag but a slightly higher failure
rate (1.4% vs 0% in our samples).

The playbook will be corrected to reflect this.

## Per-bag improvements that ARE real

Even though the *aggregate* metrics tie trial 22, trial 6 has
**substantially better long-bag drift** when it doesn't fail:

| bag | trial 22 median | trial 6 median (10-rep) | improvement |
|---|---|---|---|
| moving_long_bag1 | 0.068 | 0.015 | **-78%** |
| moving_long_bag3 | 0.058 | 0.023 (9/10) | -60% when works |
| moving_long_bag4 | 0.030 | 0.022 | -27% |
| moving_extra_long_bag1 | 0.136 | 0.045 | **-67%** |
| moving_extra_long_bag2 | 0.034 | 0.019 | -44% |
| turning_bag1 | 0.107 | 0.114 | +7% (within noise) |
| turning_bag2 | 0.175 | 0.115 | **-34%** |

Trial 6 produces **dramatically tighter long-bag drift** when it succeeds.
If we could eliminate the 10% bag3 failure mode, it'd be a clear win.
This is now an open investigation rather than a deployment recommendation.
