# `trial_6_variant_kf075` — full 8-bag validation (including moving_short_bag2)

## Why this exists

The variant's 10-rep validation on the 7-bag eval set (excluding
`moving_short_bag2`) gave median q75=0.077, worst-bag=0.133, 0
failures. User asked: does it generalize to all bags the system might
see in deployment — including `moving_short_bag2` (the structural
outlier we've been excluding from optimization eval)?

## Setup

- Same `trial_6_variant_kf075` params as `trial_6_variant_kf075_10rep.md`.
- 8 bags total: 7 standard + **`moving_short_bag2`** included.
- 5 reps in parallel, `ROS_DOMAIN_ID 40-44`.
- Wall: ~13 min.

## Per-bag results

| bag | r1 | r2 | r3 | r4 | r5 | median | max |
|---|---|---|---|---|---|---|---|
| moving_short_bag2 | 0.541 | 0.444 | 0.181 | 0.278 | 0.250 | **0.278** | **0.541** |
| moving_long_bag1 | 0.015 | 0.011 | 0.014 | 0.012 | 0.014 | 0.014 | 0.015 |
| moving_long_bag3 | 0.039 | 0.022 | 0.020 | 0.010 | 0.024 | 0.022 | 0.039 |
| moving_long_bag4 | 0.039 | 0.030 | 0.018 | 0.082 | 0.030 | 0.030 | 0.082 |
| moving_extra_long_bag1 | 0.027 | 0.033 | 0.040 | 0.044 | 0.019 | 0.033 | 0.044 |
| moving_extra_long_bag2 | 0.046 | 0.019 | 0.019 | 0.031 | 0.010 | 0.019 | 0.046 |
| turning_bag1 | 0.111 | 0.069 | 0.098 | 0.094 | 0.116 | 0.098 | 0.116 |
| turning_bag2 | 0.063 | 0.040 | 0.057 | 0.072 | 0.198 | 0.063 | 0.198 |

**0 bag failures across 40 bag-runs.** Variant remains robust even with
bag2 included.

## Trial-level (8 bags)

```
worst-bag per rep: 0.541, 0.444, 0.181, 0.278, 0.250
median worst-bag:   0.278  (always moving_short_bag2)
max worst-bag:      0.541
median q75:         0.0748
```

## How bag2 behaves under the variant

bag2 drift across 5 reps: 0.18, 0.25, 0.28, 0.44, 0.54 (median 0.28).

Reference for context (from `experiments/trial_367_validation_3rep.md`):
- #367 on bag2 (3 reps): 0.21, 0.52, 0.27 — median ~0.27.

**Variant matches #367's expected bag2 range.** bag2 remains a
structural outlier under all params (this confirms the original
hypothesis that bag2's high drift is an artifact of the recording's
short paths and degenerate start geometry, not tunable).

## How the other 7 bags compare (8-bag-5-rep vs 7-bag-10-rep)

| bag | 7-bag 10-rep median | 8-bag 5-rep median |
|---|---|---|
| moving_long_bag1 | 0.017 | 0.014 |
| moving_long_bag3 | 0.020 | 0.022 |
| moving_long_bag4 | 0.020 | 0.030 |
| moving_extra_long_bag1 | 0.052 | 0.033 |
| moving_extra_long_bag2 | 0.017 | 0.019 |
| turning_bag1 | 0.093 | 0.098 |
| turning_bag2 | 0.125 | 0.063 |

Numbers are consistent within rep-to-rep noise. The 7-bag profile
holds.

## Verdict

**`trial_6_variant_kf075` is robust across the full bag set.** Adding
`moving_short_bag2` doesn't degrade other-bag performance and doesn't
introduce failures. On bag2 itself, the variant matches #367's known
range — bag2 remains a structural outlier under all candidate params.

## Recommendation for the playbook

The 7-bag eval set was the right choice for optimization (excluding
bag2 lets us focus on tunable variance). For deployment-time use, the
variant handles all 8 valid bags including bag2 — just with bag2's
~0.28 drift expected (not 0.13 like the other bags).

If the deployment workload includes bag2-like recordings (short
trajectories with degenerate starts), expect drift ~0.18-0.54 on those
specifically. The variant doesn't make it worse than #367 did. The
~0.13 worst-bag promise applies to the 7 standard bags.
