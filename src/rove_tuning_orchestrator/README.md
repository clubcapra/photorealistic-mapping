# rove_tuning_orchestrator

Two-phase RTAB-Map tuner that combines simulated ground-truth and real-bag
validation. Distributed via Optuna's `JournalFileStorage` on a shared
filesystem — **no open ports anywhere**.

## The flow

```
                      ┌──────────────────────────────┐
                      │     ORCHESTRATOR  (`tune`)   │
                      │  stages A→B→C→D→E on the sim │
                      └──────────────┬───────────────┘
                                     ▼
                      ┌──────────────────────────────┐
                      │  SimEvaluator  (cheap)       │
                      │  Webots --mode validate      │
                      │  → ATE / drift_ratio         │
                      └──────────────┬───────────────┘
                                     ▼
                        phase1_sim/best_per_stage.json
                                     │
                          `promote --k N` selects top-K
                                     ▼
                      ┌──────────────────────────────┐
                      │  RealEvaluator  (expensive)  │
                      │  rtabmap_launch + bags +     │
                      │  reference.db                │
                      │  → corr_ratio, tracking_loss │
                      └──────────────┬───────────────┘
                                     ▼
                        phase2_real/summary.json
                                     │
                                     ▼
                          recommended params for deploy
```

Phase 1 explores cheaply on the sim (ground truth ATE — the most informative
metric you can get). Phase 2 takes only the top-K candidates and tests them
on real bags, scored against a cleaned-up reference RTAB-Map database via
node correspondence and tracking-loss events.

## Why staged CMA-ES (and not flat TPE)

You were right — Optuna's TPE degrades past ~15-20 dimensions. The fix is
**staged**: split the param space into ~5-7 dim sub-problems and tune them
in sequence (A: ICP core, B: ICP correspondences, C: odometry filter, D:
loop closure, E: joint refine). Inside each stage, use Optuna's `CmaEsSampler`
which scales much better than TPE on continuous sub-spaces.

Stages run in order; each new stage's `cumulative_best` carries the locked-in
best from prior stages, so each stage only varies its own ~5-7 dims. After
A-D, stage E (joint refine) opens a ±20% window around the cumulative best
and tunes everything jointly to clean up cross-stage interactions.

Stage YAMLs live in [config/search_spaces/](config/search_spaces/) — edit
freely, no code changes needed.

## Quick start (single machine)

```bash
# One-time: install deps and build the workspace.
sudo apt install -y webots ros-humble-webots-ros2 ros-humble-rtabmap-launch xvfb
pip install --user 'optuna>=3.4' optuna-dashboard cma pyyaml numpy

colcon build --packages-select rove_sim_webots rove_tuning_orchestrator \
             --symlink-install
source install/setup.zsh

# Phase 1: stage A→B→C→D→E on the sim, 20 trials per stage.
python3 -m rove_tuning_orchestrator.orchestrator \
  --root ~/tuning_studies \
  --project capra_v1 \
  --n-trials-per-stage 20 \
  --sampler cma_es

# Phase 2: take top-5 candidates, run them on real bags.
python3 -m rove_tuning_orchestrator.promote \
  --root ~/tuning_studies \
  --project capra_v1 \
  --k 5 \
  --bags ~/bags/moving_long_bag1 ~/bags/turning_bag1 ~/bags/moving_extra_long_bag2 \
  --reference ~/bags/reference_clean.db

# See what won.
cat ~/tuning_studies/capra_v1/phase2_real/summary.json | jq '.recommended'

# Watch it live in a browser:
python3 -m rove_tuning_orchestrator.dashboard \
  --root ~/tuning_studies --project capra_v1
# Open http://localhost:8080
```

Per-machine deploy script: [scripts/deploy_worker.sh](scripts/deploy_worker.sh)
handles the apt installs, pip installs, and colcon build, then launches a
worker.

## Distributed (no open ports)

Two practical setups, both **without exposing any port**:

### A. Shared filesystem (NFS / SSHFS / corporate share)

```bash
# On each machine:
sudo mount -t nfs nas:/srv/tuning /shared
./scripts/deploy_worker.sh \
  --root /shared/studies \
  --project capra_v1 \
  --stage stage_a_icp_core \
  --seed 11 --domain 122 --max-trials 50
```

Run one worker per stage per machine. They all write into the same journal;
Optuna's file-based locking handles concurrent appends. As soon as the
machine has 4+ cores, increase `--max-trials` and run multiple stages in
parallel.

### B. syncthing-mirrored folder (no network mount, no ports)

```bash
# On every machine:
syncthing -browser-only=false  # set up to sync ~/tuning_studies
# Then exactly as above:
./scripts/deploy_worker.sh --root ~/tuning_studies --project capra_v1 ...
```

Syncthing replicates the journal file in near-real-time using direct device-
to-device connections (no ports forwarded, no relay). Optuna treats each
machine's local copy as authoritative; conflict resolution is handled at
the journal-append level by Optuna's record-based protocol. The only risk
is if two machines append simultaneously while disconnected — both records
will sync once reconnected and Optuna replays them.

**Important:** put `<project>/phase1_sim/artifacts/` in syncthing's
`.stignore`. Per-trial bag files (~100-200 MiB each) shouldn't be replicated.

## Project layout

```
<root>/
  <project>/
    configs/                      # frozen copy of the YAML used (for repro)
    phase1_sim/
      stage_a_icp_core.journal    # one Optuna study per stage
      stage_b_icp_corresp.journal
      stage_c_odom_filter.journal
      stage_d_loop_closure.journal
      stage_e_joint_refine.journal
      best_per_stage.json         # cumulative best params (passed forward)
      summary.json                # phase-1 final
      artifacts/<stage>/<trial>/  # bag, rtabmap.db, validation.json, runner.log
    phase2_real/
      candidates.json             # top-K params lifted from phase1
      eval/candidate_NN/<bag>/    # per-bag rtabmap.db + reference_compare.json
      summary.json                # phase-2 ranked results
```

## Metrics, briefly

### Phase 1 (sim → SimEvaluator)

| Metric | What it captures |
|---|---|
| `drift_ratio` | `final_pose_error / trajectory_length`. Headline score. |
| `ate_rmse_m` | RMS aligned position error, in meters. |
| `pair_ratio` | What fraction of GT samples had a matched estimate. <30% = tracking loss → failed trial. |
| `tracking_loss_penalty` | Per-missing-pose penalty added to `score`. |

Score = `drift_ratio + tracking_loss_penalty * missing_poses`. Minimised.

### Phase 2 (real → RealEvaluator)

Score = `corr_weight*(1-correspondence_ratio) + loss_weight*tracking_loss_ratio`.
Minimised. Aggregated across bags using `median` by default.

| Metric | What it captures |
|---|---|
| `correspondence_ratio` | Fraction of candidate nodes within `tau_m` (default 0.5m) of a reference-db node. |
| `tracking_loss_ratio` | Fraction of candidate nodes with null/identity pose. |
| `mean_nn_distance_m` | Diagnostic: average distance from candidate node to nearest reference node. |

## What's not in this v1

- **Map-vs-geometry validation** (chamfer distance against the world's actual
  surfaces) — see [rove_sim_webots/rove_sim_webots/map_validator.py](../rove_sim_webots/rove_sim_webots/map_validator.py).
  Stub interface present; implement when trajectory metrics stop being the
  bottleneck.
- **GPU-aware worker scheduling** — workers are CPU-only by default; if you
  add a GPU box, just give it more `--max-trials`.
- **Automatic n_jobs scaling** — workers run one trial at a time on purpose
  (each sim+rtabmap pegs ~2 cores). Run more workers, not more concurrency
  per worker. See feedback-tuner-njobs in memory for the reasoning.
- **Integration with the existing `rove_rtabmap_tuner` package.** This is a
  parallel system: the existing tuner stays valid for offline-only studies
  on real bags. To migrate a project, feed it the same bag list + reference.

## Smoke test

```bash
# 1 trial of stage A only — finishes in ~5 min on idle hardware.
python3 -m rove_tuning_orchestrator.worker \
  --root /tmp/smoke --project smoke \
  --stage stage_a_icp_core --max-trials 1 --seed 1 --sim-domain-id 135
ls /tmp/smoke/smoke/phase1_sim/
```
