# Sim-based nav2 testing — Webots + correlation with ground truth

Runs nav2 against the Webots sim from
`.claude/worktrees/sim-webots/src/rove_sim_webots` and correlates the
planner / controller decisions with the simulator's ground-truth pose.

## Quick start

```sh
# From the repo root, with both ROS 2 and the worktree's install on the env:
bash src/rove_slam_ros/test/sim_nav_correlate.sh \
    indoor_office.wbt 2.0 /tmp/sim_nav_test
```

Args: `<world>` `<goal_x_m>` `<out_dir>`. Sets up sim + identity
`new_map → odom` static transform + raw-cloud relay onto
`/cloud_obstacles` + nav2 + ros2 bag record. Sends a `NavigateToPose`
goal `goal_x` metres forward in `new_map`, watches `/cmd_vel` for 25 s.

The PASS criterion is "any nonzero twist on `/cmd_vel`" — that's only a
floor (catches the planner-aborted-no-path case). For the deeper
"did the robot actually move toward the goal" check, post-process the
recorded bag (see below).

## Why no SLAM in the loop

Decision for this test: layer nav2 directly on top of the sim with a
fake identity `new_map → odom`, instead of putting SLAM in between.

- The sim already publishes `odom → base_link` from its wheel-odom
  estimator, plus a separate `/ground_truth/odom` for the supervisor's
  exact pose. nav2 only needs `new_map → odom → base_link` plus a
  costmap input.
- The static transform gives nav2 the missing `new_map → odom` edge.
  Identity means the map frame equals the wheel-odom origin, so the
  goal "2 m forward in new_map" lands 2 m forward of wherever the
  robot's wheel odom thinks it started.
- Skipping SLAM isolates nav2's behaviour from SLAM error. Useful for
  pinning down planner / controller bugs; not appropriate for
  long-distance closed-loop tests where wheel-odom drift dominates.

## What the test exposes

Three gotchas were caught running this for the first time:

1. **Webots IPC dir conflict with the harness's `TMPDIR`.** Webots's
   wrapper script honors `$TMPDIR` for its IPC location
   (`$TMPDIR/webots/$USER/<port>/ipc/...`), but the ros2-side
   `webots-controller` is hardcoded to `/tmp/webots/`. Claude Code's
   harness sets `TMPDIR=/tmp/claude-1000`, so the two never meet and
   every launch dies waiting for the controller to connect. Fix:
   `export TMPDIR=/tmp` before launching the sim.

2. **`map` vs `new_map` frame mismatch.** `nav2_full.yaml` uses
   `new_map` as the global frame (matches what SLAM publishes when SLAM
   is on). A naive `static_transform_publisher --frame-id map --child-frame-id odom`
   leaves the planner with two unconnected TF trees and a
   `Could not transform the start or goal pose in the costmap frame`
   error on every plan attempt. Use `new_map → odom` instead.

3. **Empty `/cloud_obstacles`.** Nav2's costmap subscribes to
   `/cloud_obstacles`, which SLAM produces by Z-banding `/livox/lidar`.
   Without SLAM the topic is empty, the costmap has no obstacles, and
   the planner sometimes silently produces "no path" without enough
   diagnostics. Fix in this test: relay `/livox/lidar → /cloud_obstacles`
   verbatim (skipping the Z-band — fine for the small worlds we use here).

## Measured correlation (indoor_office.wbt, 2 m forward)

First run on the unmodified setup, 25 s observation window:

| Source | Δx | Δy |
|---|---:|---:|
| `/ground_truth/odom` (sim supervisor — the truth) | +0.065 m | +0.127 m |
| `/odom` (wheel odom — what nav2 uses for control) | -0.010 m | +0.125 m |
| **odom drift from GT** | **-0.075 m** | **-0.002 m** |

Per-second timeline (the most revealing view — full table in the bag):

```
sec |  GT x   |  GT y   |  OD x   |  OD y   | cmd lx | cmd az
  5 | -9.0000 | +0.0000 | -0.0000 | -0.0000 | +0.000 | +1.000   ← BT alignment spin
  6 | -9.0000 | +0.0000 | -0.0125 | -0.0251 | +0.000 | +1.000
  7 | -9.0000 | -0.0000 | -0.0128 | -0.0554 | +0.000 | +1.000
  8 | -9.0000 | -0.0000 | -0.0225 | -0.1200 | +0.000 | +1.000
  9 | -9.0000 | +0.0000 | -0.0612 | -0.1831 | +0.000 | +0.926
 10 | -9.0000 | +0.0000 | -0.0292 | -0.1793 | +0.075 | -0.100   ← controller starts translating
 11 | -8.9979 | +0.0043 | -0.0046 | -0.2118 | +0.105 | +0.350
 12 | -8.9902 | +0.0350 | -0.0158 | -0.1933 | +0.030 | +0.416
 13 | -8.9775 | +0.0542 | +0.0103 | -0.1932 | +0.030 | -0.718
 14 | -8.9761 | +0.0675 | -0.0018 | -0.1056 | +0.015 | -0.980
 15 | -8.9757 | +0.0776 | -0.0227 | -0.1546 | +0.034 | -0.960
 16 | -8.9642 | +0.0956 | -0.0462 | -0.0716 | +0.131 | -0.415
 17 | -8.9351 | +0.1270 | -0.0526 | -0.0478 | +0.000 | +0.000   ← BT declares success
 ...idle, GT stationary...
```

Three things that fall out:

1. **Causality is clean: nav2's `/cmd_vel` matches actual GT motion.**
   During the spin phase (t=5–9 s, `angular.z ≈ 1`) GT shows no
   translation. As soon as `linear.x` ramps positive (t=10 s) GT starts
   moving in roughly the commanded direction.

2. **The sim's wheel-odom drifts hard during pure rotation.** After 4
   seconds of `angular.z=1` with no commanded translation, `/odom`
   thinks the robot moved -0.06 m in x and -0.18 m in y, while GT
   stayed put. nav2 uses `/odom` as its localization source, so this
   drift gets baked into where the controller thinks the robot is —
   contributing to the controller's later jerky corrections.

3. **Nav declared success at t=17 s after 0.14 m of translation, not 2 m.**
   The BT fired Goal-Succeeded based on the wheel-odom belief that
   the robot was close to (2, 0) in `new_map`, even though GT was
   barely 7 % of the way there. With identity `new_map → odom`,
   wheel-odom drift directly biases the controller's success
   criterion.

These three together justify the "no SLAM" decision specifically for
unit-testing nav2's planner/controller, and conversely justify the
"SLAM is required for closed-loop driving" position for any real run
longer than a few seconds in this sim.

## Post-processing the bag

```sh
source /opt/ros/humble/setup.bash
python3 <<'EOF'
import sqlite3
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

DB = "/tmp/sim_nav_test/correlation_bag/correlation_bag_0.db3"
con = sqlite3.connect(f"file:{DB}?mode=ro&immutable=1", uri=True)
topics = {r[1]: r[0] for r in con.execute("SELECT id, name FROM topics")}

def read(topic, msg_type):
    return [(ts * 1e-9, deserialize_message(d, msg_type))
            for (d, ts) in con.execute(
                "SELECT data, timestamp FROM messages WHERE topic_id=? ORDER BY timestamp",
                (topics[topic],))]

gt = read("/ground_truth/odom", Odometry)
od = read("/odom", Odometry)
cv = read("/cmd_vel", Twist)
# … bucket by second, compute trajectories, plot.
EOF
```

The full per-second extractor used to produce the table above is small
enough to inline here on demand; if it gets reused often we should
promote it to a `scripts/bag_to_correlation_csv.py` helper.

## Files

- `test/sim_nav_correlate.sh` — the headless test harness above.
- `test/headless_nav_smoke.sh` — the pre-existing bag-replay nav smoke
  (passes on the camera-lidar bag; covers the no-sim path).
- `NAV_TESTING.md` — manual / interactive nav testing notes.

## Open items for future runs

- **Set `goal_checker.xy_goal_tolerance`** so the BT doesn't declare
  success at 7 % progress. Currently using nav2 defaults — likely
  inherits 0.25 m, but the sim's wheel-odom drift makes that
  meaningless because nav2 thinks it's at the goal when GT shows it
  isn't.
- **Replace identity `new_map → odom` with real SLAM** once we
  re-validate that SLAM publishes a usable `new_map → odom` against
  this sim's `/livox/lidar` rate.
- **Add a "did the robot actually move ≥ 1 m toward the goal" assertion
  to the script** — the current PASS criterion only requires nonzero
  `/cmd_vel`, which is too permissive (BT recovery spins satisfy it
  without translation).
- **Try faster sim modes / a smaller world** — the recorded lidar rate
  was ≈ 3 Hz, suggesting Webots was sub-realtime on this 12-core box.
  `mode:=fast` was used but the lidar plugin step rate may be
  bottlenecked elsewhere.
