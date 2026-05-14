# What's in `tuner-autonomous-improvements`

A focused branch of stability + diagnostic + noise-reduction improvements
made during a 3-hour autonomous block. None of these change the optimizer's
core algorithm — TPE on the same search space — but they make it more
reliable and reveal what's actually limiting tuning quality.

## High-level summary

1. **Stability**: clean shutdown actually cleans up subprocesses and DB state.
2. **Diagnostics**: new `analyze_per_bag` CLI surfaces per-bag patterns; new
   playbook documents findings.
3. **Robustness**: `q75` / `q90` aggregators for noise-tolerant scoring;
   `--n-reps-per-trial` for noise-reduction via averaging.
4. **Findings**: 5× variance on identical reruns documented — many "best"
   trials may be lucky outliers. Each bag has a different best param set
   (12 distinct winners across 13 bags). Trial #367 is the most robust
   deployment candidate.

## Commits, in order

| # | summary |
|---|---|
| 1 | `runner: clean shutdown — subprocess registry + signal handler + startup self-heal` — Ctrl-C/SIGTERM now actually kills rtabmap children and cleans stale DB rows + incomplete trial dirs. |
| 2 | `optimizer: add q75/q90 quantile aggregators` — between median and max; `q75_drift_per_path` ignores 2-3 catastrophic bags without being dominated by 1. |
| 3 | `optimizer: analyze_per_bag CLI for read-only worst-bag diagnostics` — surfaces blocker bags, per-bag achievable floor, no-universal-champion check. |
| 4 | `docs: tuning playbook + deployment-ready params from trial #367` — TUNING_PLAYBOOK.md, including ready-to-paste --set lines for the most robust trial. |
| 5 | `template: add Icp/Force3DoF + Icp/Force4DoF placeholders (overridable)` — opt-in via `--set` without forcing a fresh study. |
| 6 | `tests: unit tests for quantile aggregator + cleanup helper + import audit` — 12 tests, all pass. |
| 7 | `docs: per-motion-pattern cluster patterns in winning trials` — short / long / extra-long / turning bags want measurably different params. |
| 8 | `experiments: A/B test of icp_force_3dof — bag-dependent, plus non-determinism finding` — Force3DoF helps long bag, hurts short bag. Initial non-det signal. |
| 9 | `docs: README — add playbook link + CLI quick-reference table` |
| 10 | `docs: archive per-bag winners snapshot from capra_full_v1 study` |
| 11 | `experiments: 3-rep non-determinism quantification — 5x variance confirmed` — drift_m varies 0.6 → 2.9 on identical reruns. Quantitative evidence. |
| 12 | `optimizer: --n-reps-per-trial flag for noise-reduction via averaging` — directly addresses the variance finding. |
| 13 | `docs: playbook recipe for --n-reps-per-trial usage` |

## Files of note

```
src/rove_rtabmap_tuner/
├── TUNING_PLAYBOOK.md                   ← findings + deployment recipes
├── BRANCH_NOTES.md                      ← this file
├── README.md                            ← updated with new CLIs
├── rove_rtabmap_tuner/
│   ├── analyze_per_bag.py               ← new CLI: worst-bag diagnostics
│   ├── optimizer.py                     ← q75/q90, --n-reps, signal handler
│   ├── trial_runner.py                  ← subprocess registry + cleanup_orphan_trials
│   ├── template_renderer.py             ← Force3DoF/4DoF defaults
│   └── templates/
│       └── lidar3d_tunable.launch.py.tmpl  ← Force3DoF/4DoF placeholders
├── experiments/
│   ├── force_3dof_ab.md                 ← single-rep A/B comparison
│   ├── nondeterminism_3rep.md           ← 5x variance evidence
│   └── per_bag_winners_capra_full_v1.md ← snapshot of per-bag winners
├── tests/
│   └── test_aggregators.py              ← unit tests
└── setup.py                             ← new entry_point: analyze_per_bag
```

## How to use the new bits

### Resume the existing study with q75 instead of max

```bash
ros2 run rove_rtabmap_tuner optimize \
  --bag /home/iliana/bags/moving_short_bag2 \
  --bag /home/iliana/bags/moving_long_bag1 \
  --bag /home/iliana/bags/moving_long_bag3 \
  --bag /home/iliana/bags/moving_long_bag4 \
  --bag /home/iliana/bags/moving_extra_long_bag1 \
  --bag /home/iliana/bags/moving_extra_long_bag2 \
  --bag /home/iliana/bags/moving_extra_long_bag4 \
  --bag /home/iliana/bags/turning_bag1 \
  --bag /home/iliana/bags/turning_bag2 \
  --output-root /home/iliana/prog/study_q75 \
  --study-name capra_q75_v1 \
  --metric q75_drift_per_path \
  --n-trials 100 --n-jobs 4 --n-reps-per-trial 3 \
  --seed 42 \
  --warmup-s 8 --drain-s 3 --shutdown-timeout-s 20 \
  --max-bag-duration-s 300 --expected-update-rate 50.0 \
  --bag-play-arg=--topics --bag-play-arg=/livox/lidar --bag-play-arg=/imu/data \
  --bag-play-arg=/tf --bag-play-arg=/tf_static
```

### Diagnose any study

```bash
ros2 run rove_rtabmap_tuner analyze_per_bag /home/iliana/prog/study_full
```

### Deploy trial #367 (current best)

See `TUNING_PLAYBOOK.md` § "TL;DR — recommended deployment params" for the
full `--set` list.

## What I deliberately did NOT do

- Push the branch to `origin`. The branch is local, ready when you are.
- Add `icp_force_3dof` to SEARCH_SPACE (would force a fresh study; the A/B
  test showed it's bag-dependent anyway, not a global win).
- Modify the live `capra_full_v1` DB. All changes operate on new studies
  or are read-only against the existing one.
- Fix RTAB-Map's non-determinism upstream (out of scope; documented as a
  followup).

## Suggested next steps for the human

1. **Validate trial #367 with `--n-reps-per-trial 5`** on the full bag set
   to get a real confidence interval. ~30-40 min wall.
2. **Resume with q75 aggregator + n_reps=3** for ~100 more trials. Now
   you're optimizing in low-noise mode against a robustness-friendly
   metric. Should give honest improvements over #367 if any exist.
3. **If still stuck**: ground-truth ATE (GPS / fiducials) is the structural
   unblock. Or per-bag-type tuning (the cluster pattern in the playbook).
