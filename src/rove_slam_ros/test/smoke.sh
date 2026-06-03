#!/usr/bin/env bash
# Smoke test: rebuild → source → bag-replay launch → assert /odom + /cloud_obstacles
# + /costmap/costmap topics are publishing. Exits 0 on success.
#
# Usage:  bash src/rove_slam_ros/test/smoke.sh [bag_dir]
set -uo pipefail

BAG="${1:-/home/iliana/bags/moving_extra_long_bag2}"
WS="$(cd "$(dirname "$0")/../../.." && pwd)"
cd "$WS"

echo "== sourcing ROS 2 + workspace =="
set +u; . /opt/ros/humble/setup.bash; . install/setup.bash; set -u

echo "== launching SLAM + nav2 + bag replay (background) =="
ros2 launch rove_slam_ros bag_replay.launch.py \
    bag:="$BAG" \
    with_nav:=true \
    > /tmp/smoke_launch.log 2>&1 &
LAUNCH_PID=$!
trap "kill -INT $LAUNCH_PID 2>/dev/null; wait $LAUNCH_PID 2>/dev/null" EXIT

echo "  pid=$LAUNCH_PID  waiting 20s for everything to settle…"
sleep 20

fail=0
check_topic() {
  local topic="$1" min_hz="$2"
  local hz_line
  hz_line=$(timeout 6 ros2 topic hz "$topic" 2>&1 | grep -m 1 "average rate" || true)
  if [[ -z "$hz_line" ]]; then
    echo "  ✗ $topic : NOT PUBLISHING"; fail=1; return
  fi
  local hz
  hz=$(echo "$hz_line" | awk '{print $3}')
  echo "  $topic : $hz Hz (need ≥ $min_hz)"
  awk "BEGIN{exit !($hz >= $min_hz)}" || { echo "    ↳ below threshold"; fail=1; }
}

echo "== checking topic rates =="
check_topic /odom 5.0
check_topic /cloud_obstacles 1.0
check_topic /tf 5.0
check_topic /costmap/costmap 0.5

echo "== topic list (first 25) =="
ros2 topic list 2>&1 | head -25

if [[ $fail -eq 0 ]]; then
  echo
  echo "ALL CHECKS PASSED"
  exit 0
else
  echo
  echo "SMOKE TEST FAILED — recent launch log:"
  tail -30 /tmp/smoke_launch.log
  exit 1
fi
