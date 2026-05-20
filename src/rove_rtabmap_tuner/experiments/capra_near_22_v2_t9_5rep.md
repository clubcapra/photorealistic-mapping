# `capra_near_22_v2` trial 9 — 5-rep validation (clean Pareto candidate)

## Why this run exists

`capra_near_22_v2` trial 8 had the best in-optim q75 (0.0755) but failed
5-rep validation on `moving_long_bag3` in 1 of 5 reps. Trial 9 was the
runner-up (in-optim q75=0.092) — testing whether it's more robust.

## Trial 9 params (vs trial 22 and trial 8)

| param | trial 9 | trial 8 | trial 22 |
|---|---|---|---|
| `icp_voxel_size` | 0.055 | 0.034 | 0.044 |
| `icp_max_correspondence_distance` | 0.110 | 0.101 | 0.094 |
| `icp_iterations` | 7 | 8 | 10 |
| `icp_outlier_ratio` | 0.092 | 0.108 | 0.129 |
| `icp_max_translation` | 0.426 | 0.274 | 0.398 |
| `icp_point_to_plane_k` | 33 | 33 | 27 |
| `odom_scan_keyframe_thr` | 0.820 | 0.600 | 0.722 |
| `mem_stm_size` | 13 | 10 | 10 |
| `rgbd_linear_update` | 0.451 | 0.432 | 0.420 |
| `rgbd_proximity_path_max_neighbors` | 6 | 6 | 4 |

Trial 9 uses coarser voxel (0.055 vs 0.034 for trial 8) and looser
keyframe threshold (0.820 vs 0.600). Closer to trial 22's operating
point than trial 8 was.

## 5-rep results

| bag | r1 | r2 | r3 | r4 | r5 | median | max |
|---|---|---|---|---|---|---|---|
| moving_long_bag1 | 0.134 | 0.052 | 0.035 | 0.010 | 0.020 | 0.035 | 0.134 |
| moving_long_bag3 | 0.013 | 0.027 | 0.039 | 0.016 | 0.015 | 0.016 | 0.039 |
| moving_long_bag4 | 0.033 | 0.045 | 0.052 | 0.040 | 0.015 | 0.040 | 0.052 |
| moving_extra_long_bag1 | 0.059 | 0.064 | 0.067 | 0.075 | 0.028 | 0.064 | 0.075 |
| moving_extra_long_bag2 | 0.012 | 0.048 | 0.031 | 0.021 | 0.018 | 0.021 | 0.048 |
| turning_bag1 | 0.133 | 0.122 | 0.117 | 0.084 | 0.095 | 0.117 | 0.133 |
| turning_bag2 | 0.197 | 0.032 | 0.119 | 0.202 | **0.388** | 0.197 | **0.388** |

**0 bag failures across 35 bag-runs.**

## Trial-level

```
worst-bag per rep:  0.197 / 0.122 / 0.119 / 0.202 / 0.388
median worst-bag:   0.197
max worst-bag:      0.388
q75 per rep:         0.133 / 0.058 / 0.092 / 0.080 / 0.061
median q75:          0.0795
```

## Comparison vs known candidates

| metric (5-rep median) | trial 22 | trial 18 | trial 8 (v2) | **trial 9 (v2)** |
|---|---|---|---|---|
| median q75 | 0.087 | **0.078** | 0.086 | **0.0795** |
| median worst-bag | **0.177** | 0.207 | 0.215 (FAIL) | 0.197 |
| max worst-bag (5 reps) | **0.258** | 0.331 | 1.0 (FAIL) | 0.388 |
| bag failures | 0 | 0 | 1 | **0** |

**Trial 9 = Pareto-equivalent to trial 18**: essentially same q75 (0.0795
vs 0.078), similar worst-bag (0.197 vs 0.207), no failures. The
per-bag profile differs slightly (trial 9 better on long bags, trial 18
better on `turning_bag2`).

## What we now know about the basin

Three near-22 candidates with q75 ≈ 0.08 (5-rep validated):
- **trial 18** (capra_near_22_v1): q75=0.078, worst=0.207, max=0.331
- **trial 9 (v2)** (capra_near_22_v2): q75=0.0795, worst=0.197, max=0.388
- **trial 22** (capra_focused_v3): q75=0.087, worst=0.177, max=0.258 ← still best on robustness

The q75 floor in the `near_22` basin is **~0.08**, repeatable across
different runs with different priors. **No params produce a worst-bag
below ~0.18 reliably** — this is a property of RTAB-Map's
non-determinism, not the search.

## Verdict

- **trial 22 remains the deployment default** (best max-aggregation
  robustness).
- **trial 9 (v2)** joins trial 18 as a q75-optimized Pareto-alt — pick
  either if minimizing typical-bag drift matters more than worst-case
  reliability.
- The basin around trial 22 has been thoroughly searched. Further q75
  improvement is unlikely without changing the optimizer (e.g.,
  per-motion-pattern tuning, or different metric formulation that
  attacks the per-rep variance directly).
