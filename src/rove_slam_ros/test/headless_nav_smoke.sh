#!/usr/bin/env bash
# Headless nav2 smoke test: SLAM + full nav2 + bag replay → send a goal →
# assert /cmd_vel publishes a non-zero twist. Exits 0 on PASS, 1 on FAIL.
#
# Usage:
#   bash src/rove_slam_ros/test/headless_nav_smoke.sh [bag_dir] [goal_x]
#
# The lifecycle-manager race is handled by `wait_for_nav.py` which retries
# stuck configure/activate transitions until everything reports `active`
# (or 45 s timeout).

set -uo pipefail
BAG="${1:-/home/iliana/bags/moving_extra_long_bag2}"
GOAL_X="${2:-2.0}"
# Most bags publish lidar on /livox/lidar already. The dual-lidar camera
# bag (rosbag2_test_camera_lidars) splits it across
# /livox/lidar_192_168_2_40 + /livox/lidar_192_168_2_41 — pick a primary.
BAG_LIDAR_TOPIC="${3:-/livox/lidar}"
WS="$(cd "$(dirname "$0")/../../.." && pwd)"
cd "$WS"

echo "[smoke] sourcing ROS 2 + workspace"
set +u; . /opt/ros/humble/setup.bash; . install/setup.bash; set -u

echo "[smoke] killing any leftover nav / SLAM processes"
pkill -9 -f "rove_slam_node|nav2_|component_container|lifecycle_manager|ros2 bag play" \
    2>/dev/null || true
sleep 2

echo "[smoke] launching SLAM + nav2 + bag replay (bag=$BAG, lidar=$BAG_LIDAR_TOPIC)"
# --loop keeps TF + sensor publishing alive for the whole smoke even on
# short bags (the camera-lidar bag is only 27.5 s).
ros2 launch rove_slam_ros bag_nav_bringup.launch.py \
    bag:="$BAG" rate:=1.0 loop:=true bag_lidar_topic:="$BAG_LIDAR_TOPIC" \
    > /tmp/headless_smoke.log 2>&1 &
LAUNCH_PID=$!
trap 'kill -INT $LAUNCH_PID 2>/dev/null
      sleep 2
      pkill -9 -f "rove_slam_node|nav2_|component_container|lifecycle_manager|ros2 bag play" 2>/dev/null
      wait $LAUNCH_PID 2>/dev/null
      true' EXIT

echo "[smoke] waiting 12 s for processes to come up"
sleep 12

echo "[smoke] running wait_for_nav (retries lifecycle race)"
python3 install/rove_slam_ros/lib/rove_slam_ros/wait_for_nav.py --timeout 45 \
    > /tmp/headless_wait.log 2>&1
WR=$?
tail -3 /tmp/headless_wait.log
if [[ $WR -ne 0 ]]; then
  echo "[smoke] FAIL — wait_for_nav timed out"
  exit 1
fi

echo "[smoke] sending NavigateToPose goal (${GOAL_X} m forward)"
GOAL_RESP=$(timeout 5 ros2 action send_goal /navigate_to_pose \
    nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: 'new_map'},
             pose: {position: {x: ${GOAL_X}, y: 0.0, z: 0.0},
                     orientation: {w: 1.0}}}}" 2>&1)
echo "$GOAL_RESP" | head -3
if ! echo "$GOAL_RESP" | grep -q "Goal accepted"; then
  echo "[smoke] FAIL — goal not accepted"
  exit 1
fi

echo "[smoke] watching /cmd_vel for 10 s, looking for non-zero twist"
timeout 10 ros2 topic echo /cmd_vel --no-arr > /tmp/headless_cmdvel.log 2>&1 || true
# Any non-zero linear.x OR angular.z counts as a real command.
NONZERO=$(grep -E "x:|z:" /tmp/headless_cmdvel.log | \
          awk '{print $NF}' | grep -v "^0\.0$" | grep -v "^-0\.0$" | head -3)
if [[ -n "$NONZERO" ]]; then
  echo "[smoke] PASS — /cmd_vel saw motion commands:"
  echo "$NONZERO" | head -3
  exit 0
fi
echo "[smoke] FAIL — /cmd_vel saw only zeros for 10 s"
tail -15 /tmp/headless_cmdvel.log
exit 1
