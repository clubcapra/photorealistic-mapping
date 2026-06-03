# rove_rtabmap_tuner

Automated parameter tuning for RTAB-Map: replay one or more rosbags, run RTAB-Map with different parameter sets, score each map by start-to-end pose drift, and use Optuna to search for the best parameters.

**For findings, deployment-ready params, and tuning recipes**, see [`TUNING_PLAYBOOK.md`](TUNING_PLAYBOOK.md).

## Pipeline

```
   bag(s)
     │
     ▼
┌────────────────┐    ┌──────────────────┐    ┌──────────────┐    ┌──────────────┐    ┌──────────────────┐
│ render_template│ →  │   run_trial      │ →  │ score_trial  │ →  │   optimize   │ →  │ analyze_per_bag  │
│  (per trial)   │    │ (launch + bag    │    │ (drift from  │    │ (TPE drives  │    │ (post-hoc        │
│                │    │  play + capture) │    │  rtabmap.db) │    │  run_trial)  │    │  diagnostics)    │
└────────────────┘    └──────────────────┘    └──────────────┘    └──────────────┘    └──────────────────┘
```

## Setup

Optuna is required by the optimizer but isn't in Ubuntu apt or rosdep defaults. Install it once:

```bash
pip install optuna
```

Then build like any other package in the workspace:

```bash
source /opt/ros/humble/setup.zsh
colcon build --packages-select rove_rtabmap_tuner --symlink-install
source install/setup.zsh
```

## CLIs

All exposed via `ros2 run rove_rtabmap_tuner <cmd>`. Quick reference:

| CLI | Purpose |
|---|---|
| `render_template` | Materialize the tunable launch template into a concrete `.launch.py` |
| `run_trial` | One param set vs N bags → per-bag `rtabmap.db` + `metrics.json` |
| `score_trial` | Re-score an existing trial dir (recompute drift) |
| `optimize` | Optuna-driven search over params |
| `rank_trials` | Post-hoc reranking by any registered metric |
| `analyze_per_bag` | Worst-bag / no-universal-champion diagnostics on a study dir |

### `render_template`

Renders the tunable launch template (a copy of `rove_color_mapping/launch/lidar3d.launch.py` with `${placeholder}` markers) with a set of param overrides into a concrete launch file.

```bash
ros2 run rove_rtabmap_tuner render_template --list-keys
ros2 run rove_rtabmap_tuner render_template \
  -o /tmp/trial.launch.py \
  -s icp_iterations=25 \
  -s grid_cell_size=0.08
```

### `run_trial`

Runs one parameter set against one or more bags. Each bag gets its own subdir with the rendered launch file, captured stdout/stderr, the resulting `rtabmap.db`, and a `metrics.json` (drift).

```bash
ros2 run rove_rtabmap_tuner run_trial \
  --bag /path/to/bag1 --bag /path/to/bag2 \
  --output-root ./tuning_runs \
  --trial-id baseline \
  -s icp_iterations=25 \
  --lidar-topic /livox/lidar --imu-topic /imu/data --frame-id base_link \
  --expected-update-rate 15.0 \
  --max-bag-duration-s 600
```

Useful flags:
- `--dry-run` — render launch files and write the resolved `ros2 launch` / `ros2 bag play` commands but don't execute anything.
- `--max-bag-duration-s N` — wall-clock cap on bag playback (safety net for corrupt/runaway bags).
- `--warmup-s` / `--drain-s` — seconds to wait for RTAB-Map to subscribe before bag start, and for in-flight messages to flush before SIGINT.

### `score_trial`

Re-scores all `rtabmap.db` files under a trial directory. Useful when iterating on the metric without re-running.

```bash
ros2 run rove_rtabmap_tuner score_trial ./tuning_runs/baseline
```

Writes per-bag `metrics.json` and an aggregate `trial_scores.json`.

### `optimize`

Drives `run_trial` with Optuna TPE. Stores the study in SQLite under `<output_root>/optuna.db` so runs can be resumed.

```bash
ros2 run rove_rtabmap_tuner optimize \
  --bag /path/to/bag1 --bag /path/to/bag2 \
  --output-root ./tuning_study \
  --study-name corridor_v1 \
  --n-trials 50 \
  --max-bag-duration-s 600
```

The study writes `study_summary.json` (best params + top 5) on exit. Re-run with the same `--study-name` and `--output-root` to add more trials.

Use `--list-search-space` to see what's being tuned and the ranges. Use `--objective synthetic` to smoke-test the optimizer without spawning RTAB-Map (it scores against a hidden target param vector).

### Parallel trials

`--n-jobs N` runs up to N trials concurrently. Each worker is assigned a unique `ROS_DOMAIN_ID` (1-99) from a pool, so concurrent RTAB-Map pipelines don't see each other's topics over DDS. Reserved domain IDs are always skipped:

- **0** — the default; would clash with anything else running on this machine.
- **96** — the live Rove robot. Using it would inject test traffic into production.

To use additional reserved domains, edit `RESERVED_DOMAIN_IDS` in `rove_rtabmap_tuner/optimizer.py`.

Rough sizing: each trial spawns icp_odometry + rtabmap (~1-2 cores under load, 500MB-1GB RSS). On a 12-core / 32GB+ machine, `--n-jobs 4` is comfortable. A 56-second sequential trial costs the same wall time as 4 parallel trials — i.e. you can do 4x the trials in the same time budget.

```bash
ros2 run rove_rtabmap_tuner optimize \
  --bag bag1 --bag bag2 \
  --output-root ./tuning_study \
  --n-trials 40 --n-jobs 4 \
  --max-bag-duration-s 90
```

## Accuracy metric

The scorer uses **start-to-end pose drift**: the Euclidean distance between the first and last poses of the optimized trajectory. Only meaningful for bags where the robot deliberately returned to (approximately) the starting position. GPS-based ATE is intentionally not used here.

## Recording a tuning-ready bag

```bash
ros2 bag record -a -O my_loop_run
# Drive the robot in a loop. Stop the recording AFTER returning to the start.
```

The bag must contain `/livox/lidar` and `/imu/data` (or whatever topics you pass via `--lidar-topic` / `--imu-topic`), plus `/tf` and `/tf_static`. If your bag was recorded with custom message types not built in the workspace, `ros2 bag play` will skip those topics — make sure any required message packages are installed.

## Template ↔ canonical launch drift

The template at `rove_rtabmap_tuner/templates/lidar3d_tunable.launch.py.tmpl` is a copy of `rove_color_mapping/launch/lidar3d.launch.py` with tunable values replaced by `${placeholder}` markers. If you change non-tuned bits in the canonical launch file, update the template to match.
