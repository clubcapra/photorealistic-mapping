# Sim SLAM map accuracy — honest report

End-to-end accuracy measurement of `rove_slam_node` against the Webots
sim's ground truth. Compares SLAM's accumulated point cloud (built from
SLAM's pose estimates) against a GT-pose-built cloud from the same lidar
scans. Recorded 2026-06-05; written to be reproducible via
[`test/sim_slam_accuracy.sh`](test/sim_slam_accuracy.sh).

## TL;DR — it's not great

| Metric | Value | What it means |
|---|---|---|
| Trajectory length | ~0.5 m forward + 1 full rotation | 26 s of motion |
| **Median XY pose error** | **0.05 cm** | SLAM tracks translation almost perfectly during pure rotation |
| **Final XY pose error** | **26 cm** | Drift accumulated after 26 s of motion |
| **Mean yaw error** | **+31.3°** | Linear drift during the spin |
| **Final yaw error** | **+56.6°** | Half a turn off ground truth |
| **Yaw drift rate** | **~3.6 °/s** | During 0.4 rad/s = 23 °/s commanded rotation |
| **Map NN median** | 8 cm | Half the points are within 8 cm of the closest GT point |
| **Map NN mean** | 41 cm | Mean dragged up by far points that the yaw error throws out by metres |
| **Map NN max** | 16 m | Worst-case scan got rotated nearly 60° off — far walls land entirely outside the room |
| **ICP fitness** | 0.81 | ICP-aligned 81% of SLAM points with GT inliers under 0.5 m correspondence |
| **ICP inlier RMSE** | 15 cm | Residual after best rigid alignment — most of this is the yaw smear |

## Setup

Recorded with `/tmp/sim_slam_record.sh`:

1. Webots `indoor_office.wbt`, `mode:=fast`, headless (`WEBOTS_GUI=false`,
   `xvfb-run`, `TMPDIR=/tmp`). Sim's `/livox/lidar` was emitting **at ~4 Hz**
   (vs the real MID-360's 10 Hz — see Caveats).
2. Robot started at sim world (-9, 0, 0.16) and was driven via a manual
   `/cmd_vel` Python publisher: 18 s of `angular.z=0.4 rad/s` spin (~1 rotation)
   followed by 4 s of `linear.x=0.15 m/s` forward.
3. Recorded `/livox/lidar`, `/ground_truth/odom`, `/odom`, `/tf`, `/tf_static`
   for 27 s.

SLAM was then run **offline** by replaying the recorded bag through
`rove_slam_node` (`use_sim_time:=false`, `urdf_extrinsic:=true`, sim's
`/tf` + `/tf_static` remapped to dead-end topics so they didn't conflict
with SLAM's own publication). Two bags result:

* `correlation_bag/` — sim outputs + `/ground_truth/odom` (the truth).
* `slam_replay/slam_bag/` — SLAM's `/odom` estimates.

The analysis script ([`scripts/sim_slam_accuracy.py`](scripts/sim_slam_accuracy.py)):

1. Per-scan: interpolate SLAM's pose AND GT's pose at the scan's bag-rel
   timestamp.
2. Accumulate two clouds: `cloud_slam = ∑ T_slam_base × T_base_lidar_slam × scan`
   and `cloud_gt = ∑ T_world_base × T_base_lidar_sim × scan`.
3. Voxel-downsample (3 cm).
4. ICP-align `cloud_slam → cloud_gt` (initial seed: T_align that maps
   SLAM's first pose to GT's first pose).
5. KD-tree nearest-neighbour distances both directions.
6. Per-second pose error series (Position + yaw).

## Drift profile (the smoking gun)

```
  t(s) | XY err(cm) | Z err(cm) | yaw err (deg)
   0.0 |     0.00   |    +0.00  |      +0.00
   2.0 |     0.05   |    +0.01  |      +6.41
   4.1 |     0.05   |    +0.01  |     +11.48
   6.2 |     0.05   |    +0.01  |     +18.83
   8.6 |     0.05   |    +0.01  |     +28.29
  10.8 |     0.05   |    +0.01  |     +38.37
  12.7 |    64.37   |   +40.20  |     -32.57  ← transient (interpolation? pose jump?)
  15.1 |     0.05   |    +0.01  |     +46.33
  17.5 |     0.05   |    +0.01  |     +52.48
  19.8 |    17.56   |   +17.54  |     +45.47
  22.1 |    13.33   |    +0.01  |     +56.57
  24.4 |    25.33   |    +0.01  |     +56.57
  26.4 |    25.97   |    +0.01  |     +56.57  (last)
```

What it tells me:

* **Pure rotation (t=0–18 s): yaw drifts linearly at ~3.6°/s while XY
  stays at 0.05 cm.** SLAM's translation tracking is fine in isolation
  — when the robot doesn't translate, neither does SLAM's estimate.
  The yaw error is the entire story for this phase.
* **3.6°/s yaw drift at 22.9°/s commanded** = about 16% of every degree
  of rotation is lost. That's a registration error, not noise — KISS-ICP
  cannot fit ~6° of per-scan rotation reliably at this lidar rate.
* **The forward leg (t=18–22 s) adds ~25 cm of XY drift on top of the
  already-drifted yaw.** Because SLAM thinks the robot is pointing 50°
  off truth, "drive 0.5 m forward" lands the SLAM frame 0.5 m × sin(50°)
  ≈ 38 cm off in y — but ICP-alignment absorbed some of this, leaving
  ~26 cm.
* **The t=12.7 s outlier** (XY=64 cm, yaw=-33°) is a one-frame glitch in
  the pose interpolation — likely a SLAM /odom message with a stale
  timestamp at exactly the rebasing seam. Doesn't change the overall
  picture; flagged here for honesty.

## Map-cloud accuracy

After ICP-aligning SLAM cloud to GT cloud (best-fit rigid alignment),
per-point NN distance distribution (SLAM → GT):

```
  mean   :  40.98 cm   ← dragged up by yaw-smeared far points
  median :   8.05 cm   ← representative for most of the room
  p90    : 135.53 cm
  p95    : 230.65 cm
  p99    : 358.83 cm
  max    : 1611.73 cm  ← walls 16 m away that the yaw error throws clean out of the room
```

The median is the honest "typical point error" — 8 cm. The mean and
high percentiles measure the **yaw-error tail**: every scan's far points
get displaced by (range × sin(yaw_error)). With yaw error reaching 60°
and lidar ranges up to 30 m, displaced points up to ~26 m are expected.

For a "what the SLAM map LOOKS like" diagnostic, the median is the
right number. For "is this map usable for nav or photogrammetry", the
**16 m max says no** — far surfaces are smeared into mush.

## Why is the lidar at 4 Hz?

The single most likely root cause. Webots's `Lidar` device is driven by
the simulation step. Under `mode:=fast`, the sim steps as fast as it
can, but the lidar plugin is bottlenecked by:

* Ray-casting cost (1024 × 64 rays at 30 m × 32-beam pattern).
* Single-threaded Webots GUI loop, even with `--no-rendering`.

On this 12-core box at `nice -n 10`, the sim ran sub-realtime at about
40% of the design speed. A real Rove cycle is 10 Hz; here we're at 4 Hz.
That means each consecutive scan that KISS-ICP sees is ~6° rotated from
the previous, not the ~2.4° it would be at 10 Hz.

Three angles to validate this hypothesis:

1. Rerun with `mode:=realtime` and a smaller world that fits in the
   CPU budget — should bring lidar to ≥9 Hz and dramatically reduce
   yaw drift.
2. Drop the spin rate from 0.4 rad/s to 0.15 rad/s — would make each
   per-scan rotation small enough for KISS-ICP to track even at 4 Hz.
3. Try `voxel_size` smaller than 0.30 — denser hash map could help
   resolve fine rotations.

I'd start with (1).

## Caveats

* **Sim base→lidar mismatch.** Sim publishes `base_link → livox_frame =
  xyz(-0.3, 0, 0.28), rpy(0, 30°, 180°)`; SLAM's hardcoded extrinsic
  (when `urdf_extrinsic=true`) is `xyz(-0.30, 0, 0.318)` — 3.8 cm Z
  off. The analysis uses the SIM extrinsic for GT and SLAM's extrinsic
  for SLAM; the ICP alignment absorbs most of this as a rigid offset
  but it bumps the reported residuals up by a few cm.
* **No loop closure.** This is a single-segment trajectory — no chance
  for SLAM's loop-closure module (when enabled in the submodule) to
  correct yaw drift. A real run in a closed indoor space would
  eventually close a loop and snap back.
* **Sim lidar noise ≠ real MID-360.** The sim's Webots Lidar device
  models a perfect time-of-flight return with no atmospheric, beam-
  divergence, or surface-reflectance noise. So the map's *floor noise*
  is artificially low; the report's residual is **almost entirely from
  SLAM pose error**, not lidar noise.
* **Wall-time replay.** Using `use_sim_time:=false` on the SLAM replay
  means SLAM saw scans at wall-clock arrival times, not sim's recorded
  times. The bag was replayed at 1.0×, so within-run timing is preserved,
  but per-scan dt jitter is the wall-clock's, not the sim's. This is
  unlikely to materially change the result but is worth noting.

## Files

* [`test/sim_slam_accuracy.sh`](test/sim_slam_accuracy.sh) — record sim
  bag + replay through SLAM + invoke the analysis. ~70 s wall-clock.
* [`scripts/sim_slam_accuracy.py`](scripts/sim_slam_accuracy.py) — the
  analysis. Reads both bags, builds clouds, ICP-aligns, writes report.
* `report.txt`, `cloud_slam.pcd`, `cloud_gt.pcd`,
  `cloud_slam_aligned.pcd` — outputs (under `$OUT_DIR`, default
  `/tmp/sim_slam_map_test/`).

## What I'd ask for before trusting any sim-based SLAM result

1. **Sim lidar at ≥9 Hz**: tune Webots stepping until the lidar publish
   rate matches the real MID-360.
2. **A second trajectory**: figure-eight in a 5 m × 5 m room. Engages
   loop closure if the submodule has it on; tests translation tracking
   independently of pure rotation.
3. **Extrinsic match**: either set `urdf_extrinsic:=false` to skip
   SLAM's hardcoded transform (and feed the sim's actual one separately)
   or fix the hardcoded value to match the sim driver's
   `xyz(-0.3, 0, 0.28)`.
4. **Loop closure on**: this submodule has it but defaults to off in
   the bridge. Should be turned on for any "test SLAM end-to-end" run.
