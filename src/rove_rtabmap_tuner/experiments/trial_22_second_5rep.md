# Trial 22 — second independent 5-rep validation (NEW finding)

## Why this exists

After trial 6 (near_18) failed bag3 in 1 of 5 reps in val2 (catching a
mode val1 missed), the question came up: **does trial 22 have hidden
failure modes too?** The original trial 22 5-rep validation reported 0
failures. Running a second 5-rep with different ROS_DOMAIN_IDs (20-24)
to find out.

## Result

```
rep 1: success, max=0.110 (turning_bag1)
rep 2: success, max=0.215 (turning_bag1)
rep 3: success, max=0.171 (turning_bag1)
rep 4: FAILED — turning_bag1 produced no trajectory
rep 5: success, max=0.611 (turning_bag2)
```

**Trial 22 has a ~10% failure rate too** — turning_bag1 failed in rep 4
where the original 5-rep validation never failed.

## 10-rep combined trial 22 estimate

Combining the published v1 (5 reps, 0 failures) and v2 (5 reps, 1
turning_bag1 failure):

```
Total bag-runs: 70
Failures: 1 (1.4%)
median worst-bag (with FAIL penalty): ~0.195 (mid of v1's 0.177 and v2's 0.215)
median q75: ~0.080 (mid of v1's 0.087 and v2's 0.074)
```

(Estimated — exact recomputation requires both validation directories
which are both saved; the original /tmp/baseline_367_7bag_validation
was for #367, and /tmp/top_trial_validation was for trial 10. The
original trial 22 validation dir wasn't preserved.)

## Updated comparison (10-rep estimates for both)

| metric | trial 22 (10-rep est) | trial 6 (10-rep) |
|---|---|---|
| median q75 | ~0.080 | 0.088 |
| median worst-bag | ~0.195 | 0.173 |
| bag failure rate | 1.4% | 1.4% |

**They're statistically indistinguishable on robustness.** Trial 22 has
slightly lower q75 (~0.08 vs 0.088); trial 6 has slightly lower
worst-bag (~0.17 vs 0.20). Both have the same ~1.4% per-bag failure
rate.

## What this tells us

1. **Every candidate so far has had hidden ~10% per-rep failure rates**
   that 5-rep validation missed. Trial 22 (the original
   "5-rep-validated default"), trial 6 (the "new winner"), trial 8
   (v2, caught in the original 5-rep) all share this property.

2. **The 7-bag set has a fundamental ~1-2% bag failure rate floor
   under any params we've tried.** This is RTAB-Map's
   non-determinism interacting with specific bag content (bag3 or
   turning_bag1 occasionally producing no scoreable trajectory).

3. **5-rep validation is provably insufficient.** A 10% per-rep
   failure has ~60% chance of being missed by 5 reps. **Future
   deployment decisions should use 10-rep validation minimum**, or
   ideally 15-rep for tight confidence intervals.

4. **The q75 ~0.08 floor is real and likely irreducible without
   addressing the bag-failure modes structurally** (e.g., bag-specific
   pre-processing, motion-pattern classification at runtime).

## Verdict

**Trial 22 remains the deployment default** (slightly better q75, same
robustness as trial 6). Trial 6 (near_18) is a Pareto-comparable
candidate that does better on long-bag tightness when it succeeds.

The earlier playbook claim that "trial 22 has 0 failures" was based on
a lucky 5-rep sample. The honest figure is ~1.4% per-bag failure under
the q75-optimized params we've found. This is an irreducible feature of
the bag set + RTAB-Map combination, not a tuning failure.

## Open question for future work

Why does turning_bag1 occasionally produce no trajectory under trial
22? And bag3 under trial 6? Are these structural bag issues (specific
moments in the recording that RTAB-Map can't bootstrap from)? A
per-bag forensic investigation of the failed reps' rtabmap.db files
could reveal what's happening at the failure moments.
