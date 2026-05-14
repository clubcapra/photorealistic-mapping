# Per-bag winning trials from `capra_full_v1` (~700 trials)

These are the trials that achieved the lowest single-bag drift across the
entire study. Useful for understanding what params each bag prefers, and as
the basis for any future per-bag-type ensemble approach.

| bag | best trial | drift on that bag | drift_per_path |
|---|---:|---:|---:|
| bag_imobile | #39 | 0.0004 m | 0.0004 |
| moving_short_bag1 | #96 | — | 0.0072 |
| moving_short_bag2 | #583 | — | 0.0005 |
| moving_long_bag1 | #336 | — | 0.0004 |
| moving_long_bag2 | #401 | — | 0.0001 |
| moving_long_bag3 | #336 | — | 0.0008 |
| moving_long_bag4 | #709 | — | 0.0007 |
| moving_extra_long_bag1 | #538 | — | 0.0034 |
| moving_extra_long_bag2 | #369 | — | 0.0007 |
| moving_extra_long_bag3 | #429 | — | 0.0013 |
| moving_extra_long_bag4 | #36 | — | 0.0000 |
| turning_bag1 | #419 | — | 0.0110 |
| turning_bag2 | #550 | — | 0.0050 |

**12 distinct winners across 13 bags** = strong evidence that no universal
parameter set is best for all bags.

## Per-motion-pattern parameter medians (across winners in each category)

Computed from the per-winner params.json files; see `TUNING_PLAYBOOK.md`
section 3a for the inline table and interpretation.

```
                  voxel_size  corr_dist  outlier_ratio  max_translation  stm_size
short bags          0.058     0.44       0.37           1.68             7.5
long bags           0.082     0.50       0.50           1.23             8.5
extra-long bags     0.057     0.16       0.51           1.52             12.0
turning bags        0.077     0.61       0.72           1.78             8.5
```

**The actionable signal**: if you can classify bags by motion at runtime,
select params from the appropriate cluster. Long bags want bigger voxels +
larger correspondence distance; extra-long want tighter correspondence
distance + larger STM; turning bags want strict outlier rejection.

## Reproduction

The data was extracted via:
```bash
ros2 run rove_rtabmap_tuner analyze_per_bag /home/iliana/prog/study_full
```

with additional param-clustering done manually against the
`/home/iliana/prog/study_full/trial_NNNN/params.json` files for each
winning trial.

This file is a snapshot — re-run `analyze_per_bag` to get current data if
the study has changed.
