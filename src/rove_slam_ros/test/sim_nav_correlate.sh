#!/usr/bin/env bash
# Headless Webots sim + nav2 + recording for nav-decision vs actual-location
# correlation. No SLAM in the loop — identity static_transform_publisher gives
# new_map → odom so nav2 can plan, and the sim's /livox/lidar is relayed to
# /cloud_obstacles so the local costmap has obstacles to inflate around.
#
# Outputs (default: /tmp/sim_nav_test):
#   sim.log              Webots + ros2 driver
#   nav.log              nav2_bringup
#   static_tf.log        static transform publisher
#   cloud_relay.log      lidar -> cloud_obstacles relay
#   wait_for_nav.log     lifecycle-race retry helper
#   cmd_vel_stream.log   `ros2 topic echo /cmd_vel` during the goal
#   goal_resp.log        action server response to the goal
#   correlation_bag/     ros2 bag with /odom, /ground_truth/odom, /plan,
#                         /cmd_vel, /tf, /tf_static for post-analysis
#
# Exit 0 = PASS criterion (nonzero twist on /cmd_vel observed).
# Exit nonzero = FAIL (sim never produced /livox/lidar, or planner never
# emitted a path and /cmd_vel stayed zero).
#
# Important — this script handles a Claude Code harness quirk: the harness
# sets TMPDIR=/tmp/claude-1000, which Webots's launch wrapper honors for its
# IPC directory. But webots-controller (ros2 side) is hardcoded to
# /tmp/webots — mismatch breaks every launch. We `export TMPDIR=/tmp` at
# the top to force them into the same dir. See CAMERA_CALIBRATION.md for
# the analogous "two surprising-but-correct facts" pattern.

set +u

WORLD="${1:-indoor_office.wbt}"
GOAL_X="${2:-2.0}"
OUT="${3:-/tmp/sim_nav_test}"

# We're a sibling of src/rove_slam_ros/test, so the workspace root is two up.
WS="$(cd "$(dirname "$0")/../../.." && pwd)"
cd "$WS"

source /opt/ros/humble/setup.bash 2>/dev/null
source install/setup.bash 2>/dev/null
source .claude/worktrees/sim-webots/install/setup.bash 2>/dev/null

# Webots TMPDIR fix — see header.
export TMPDIR=/tmp

unset DISPLAY WAYLAND_DISPLAY
export ROS_DOMAIN_ID=132 WEBOTS_GUI=false WEBOTS_PORT=1234 QT_QPA_PLATFORM=xcb

rm -rf "$OUT" && mkdir -p "$OUT"
echo "[smoke] out dir: $OUT, world: $WORLD, goal_x: $GOAL_X m"

# Clean stale Xvfb locks (xvfb-run -a picks lowest free display)
for lock in /tmp/.X*-lock; do
    [ -f "$lock" ] || continue
    n=$(basename "$lock"); n=${n#.X}; n=${n%-lock}
    pgrep -f "Xvfb :$n " >/dev/null 2>&1 || rm -f "$lock" "/tmp/.X11-unix/X$n" 2>/dev/null
done
# Clean any leftover processes
pkill -9 -f 'webots|sim_webots|Xvfb|nav2_|rove_slam_node|component_container|lifecycle_manager|static_transform_publisher|topic_tools.*relay|ros2 bag record' 2>/dev/null
sleep 2
rm -rf /tmp/webots/iliana 2>/dev/null

echo "[smoke] STEP 1: launch sim"
nice -n 10 xvfb-run -a --server-args="-screen 0 1024x768x24" \
    ros2 launch rove_sim_webots sim.launch.py \
        world:="$WORLD" mode:=fast \
    > "$OUT/sim.log" 2>&1 &
SIM_PID=$!

cleanup() {
    [ -n "$REC_PID" ]   && kill -SIGINT $REC_PID 2>/dev/null
    [ -n "$NAV_PID" ]   && kill -SIGINT $NAV_PID 2>/dev/null
    [ -n "$RELAY_PID" ] && kill -SIGINT $RELAY_PID 2>/dev/null
    [ -n "$STAT_PID" ]  && kill -SIGINT $STAT_PID 2>/dev/null
    [ -n "$SIM_PID" ]   && kill -SIGINT $SIM_PID 2>/dev/null
    sleep 3
    pkill -9 -f 'webots|sim_webots|Xvfb|nav2_|rove_slam_node|component_container|lifecycle_manager|static_transform_publisher|topic_tools.*relay|ros2 bag record' 2>/dev/null
    return 0
}
trap cleanup EXIT

echo "[smoke] STEP 2: wait for /livox/lidar (sim ready)"
for i in $(seq 1 45); do
    ros2 topic info /livox/lidar 2>/dev/null | grep -q "Publisher count: [1-9]" \
        && { echo "  /livox/lidar live after ${i}s"; break; }
    sleep 1
done

echo "[smoke] STEP 3: identity static_transform_publisher new_map -> odom"
# nav2 configs use 'new_map' as the global frame (matches what SLAM
# publishes when SLAM is on). Without SLAM, we need a static identity
# transform so the planner can transform goals from new_map into the
# costmap frame.
nice -n 10 ros2 run tf2_ros static_transform_publisher \
    --frame-id new_map --child-frame-id odom \
    --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 \
    --ros-args -p use_sim_time:=true \
    > "$OUT/static_tf.log" 2>&1 &
STAT_PID=$!

echo "[smoke] STEP 4: relay /livox/lidar -> /cloud_obstacles"
# nav2's local costmap subscribes to /cloud_obstacles (normally produced
# by SLAM Z-banding /livox/lidar). Without SLAM, we just feed the raw
# cloud. Costmap inflates around obstacle returns either way.
nice -n 10 ros2 run topic_tools relay /livox/lidar /cloud_obstacles \
    --ros-args -p use_sim_time:=true \
    > "$OUT/cloud_relay.log" 2>&1 &
RELAY_PID=$!

sleep 3
echo "[smoke] STEP 5: TF chain check (new_map -> base_link)"
timeout 4 ros2 run tf2_ros tf2_echo new_map base_link 2>&1 | tail -6

echo "[smoke] STEP 6: launch nav2"
nice -n 10 ros2 launch rove_slam_ros nav_bringup.launch.py \
    use_sim_time:=true \
    > "$OUT/nav.log" 2>&1 &
NAV_PID=$!

echo "[smoke] STEP 7: wait for nav lifecycle"
for i in $(seq 1 30); do
    sleep 2
    ros2 topic info /plan 2>/dev/null | grep -q "Publisher count: [1-9]" && break
done
python3 install/rove_slam_ros/lib/rove_slam_ros/wait_for_nav.py --timeout 60 \
    > "$OUT/wait_for_nav.log" 2>&1 || true
tail -3 "$OUT/wait_for_nav.log"

echo "[smoke] STEP 8: bag-record correlation topics"
nice -n 10 ros2 bag record -o "$OUT/correlation_bag" \
    /odom /ground_truth/odom /plan /cmd_vel /tf /tf_static \
    > "$OUT/rec.log" 2>&1 &
REC_PID=$!
sleep 2

echo "[smoke] STEP 9: send NavigateToPose (${GOAL_X} m forward in new_map)"
timeout 8 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: 'new_map'},
             pose: {position: {x: ${GOAL_X}, y: 0.0, z: 0.0},
                     orientation: {w: 1.0}}}}" \
    > "$OUT/goal_resp.log" 2>&1 &

echo "[smoke] STEP 10: watch /cmd_vel for 25 s"
timeout 25 ros2 topic echo /cmd_vel --no-arr > "$OUT/cmd_vel_stream.log" 2>&1 || true

echo "[smoke] STEP 11: stop recorder"
kill -SIGINT $REC_PID 2>/dev/null
sleep 3

echo "[smoke] DONE — files in $OUT:"
ls -la "$OUT/"

# Pass criterion: any nonzero twist seen on /cmd_vel during the goal
nonzero=$(grep -E "x:|z:" "$OUT/cmd_vel_stream.log" 2>/dev/null | \
          awk '{print $NF}' | grep -v "^0\.0$" | grep -v "^-0\.0$" | head -3)
if [[ -n "$nonzero" ]]; then
    echo "[smoke] PASS — /cmd_vel nonzero (nav2 acted):"
    echo "$nonzero"
    exit 0
fi
echo "[smoke] FAIL — /cmd_vel only zeros (planner produced no usable path)"
exit 1
