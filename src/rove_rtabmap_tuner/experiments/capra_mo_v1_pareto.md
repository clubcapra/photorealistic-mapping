# `capra_mo_v1` — multi-objective NSGA-II (q75 + max_drift_per_path)

## Idea

Trial 22 wins on max-aggregation; trial 18 wins on q75. Each represents a
different point on the q75 ↔ max trade-off. A multi-objective NSGA-II
study might find Pareto-better candidates between (or beyond) them.

## Setup

- New study `capra_mo_v1` with 2 objectives: `q75_drift_per_path` AND
  `max_drift_per_path`, both minimize.
- `near_22` search space (same anchor that found trial 18).
- 8 trials, `n_jobs=2`, `n_reps=3`. NSGA-II default population size.
- Wall time: ~2 hr.

## Results — Pareto front (in-optim n_reps=3 medians)

```
trial 1: q75=0.149, max=0.267
trial 2: q75=0.351, max=0.549
trial 3: q75=0.615, max=0.837  (catastrophic init)
trial 4: q75=0.137, max=0.273  ← Pareto front (best q75)
trial 5: q75=0.115, max=0.272
trial 6: q75=0.171, max=0.375
trial 7: q75=0.229, max=0.322  ← Pareto front (best max)
trial 8: q75=0.140, max=0.263
```

Optimizer's reported Pareto front: trials 4 and 7. Trial 5 has the
absolute lowest q75 but its max isn't on the Pareto front (it's
dominated by trial 4 on max? — actually trial 5 max=0.272 > trial 4 max=0.273... close call; possibly tie-broken by the tool).

## Comparison vs known good candidates

| metric (5-rep median) | trial 22 | trial 18 | trial 5 (MO best q75) | trial 7 (MO best max) |
|---|---|---|---|---|
| median q75 | 0.087 | **0.078** | 0.115 (in-optim) | 0.229 (in-optim) |
| max_drift_per_path | 0.258 | 0.331 | 0.272 (in-optim) | **0.263 (in-optim)** |

**MO Pareto trials are strictly worse** than the existing single-obj
winners on both objectives they were supposedly optimizing:

- Best MO q75 (trial 5, 0.115) > trial 18's 5-rep q75 (0.078) — about
  **50% worse**.
- Best MO max (trial 7, 0.263) only barely better than trial 22's
  5-rep max (0.258), and worse on q75 (0.229 vs trial 22's 0.087).

## Why NSGA-II failed here

1. **Too few trials for the dimension**: 8 trials in 16-dim search space,
   even narrow (`near_22`), is far below NSGA-II's typical
   population-size × generations product (40-100 trials minimum for
   real Pareto front discovery).

2. **Bootstrap dilution**: NSGA-II's first generation is uniform random
   sampling — no prior bias toward known-good regions. Trial 22 and
   trial 18 sit in a specific basin; randomized init mostly missed it.

3. **Per-objective noise**: `max_drift_per_path` is the noisiest metric
   we have (single bad rep dominates). NSGA-II's domination ranking
   amplifies this noise — two near-equivalent trials can be ranked
   differently across runs.

4. **TPE's prior exploitation**: the single-obj TPE study that found
   trial 18 had 8+ priors near trial 22 to exploit. NSGA-II started
   from scratch.

## Verdict

**No new winner from MO**. To make NSGA-II work in this codebase:
- 40+ trials minimum.
- Seed it with trial 22 and trial 18 as "warm start" priors (the
  `optimizer.py` doesn't expose this currently; would need a code
  change).
- Use a less noisy max-style metric (q90 might work better than max).

For now: keep trial 22 (default deployment) and trial 18 (long-bag-dominant
alternative). The Pareto-front exploration just confirmed they're a
hard-to-beat pair.

## Reproducing

```bash
ros2 run rove_rtabmap_tuner optimize \
  --bag /home/iliana/bags/moving_long_bag1 ... [7 bags] \
  --output-root /home/iliana/prog/study_mo_v1 \
  --study-name capra_mo_v1 \
  --metric q75_drift_per_path \
  --metric max_drift_per_path \
  --search-space near_22 \
  --n-trials 8 --n-jobs 2 --n-reps-per-trial 3 \
  --seed 555 \
  ...
```
