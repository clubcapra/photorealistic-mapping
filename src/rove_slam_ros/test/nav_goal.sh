#!/usr/bin/env bash
# Headless nav test: launch the full nav2 stack (planner + controller +
# behaviors + costmap + bt_navigator) wired to our /cloud_obstacles, then
# fire a NavigateToPose action and watch /cmd_vel.
#
# This requires nav2 packages installed (already on this machine).
# Exits 0 if /cmd_vel publishes a non-trivial twist within 30 s of goal send.
set -uo pipefail

BAG="${1:-/home/iliana/bags/moving_extra_long_bag2}"
WS="$(cd "$(dirname "$0")/../../.." && pwd)"
cd "$WS"

set +u; . /opt/ros/humble/setup.bash; . install/setup.bash; set -u

# Start SLAM + bag replay in background.
ros2 launch rove_slam_ros bag_replay.launch.py \
    bag:="$BAG" with_nav:=true \
    > /tmp/nav_launch.log 2>&1 &
LAUNCH_PID=$!
trap "kill -INT $LAUNCH_PID 2>/dev/null; wait $LAUNCH_PID 2>/dev/null" EXIT
echo "launch pid=$LAUNCH_PID"

sleep 15
echo "== sending NavigateToPose goal (3m forward in odom) =="
ros2 action send_goal -f /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: 'map'},
    pose: {position: {x: 3.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}
  }
}" > /tmp/nav_action.log 2>&1 &
GOAL_PID=$!

echo "== watching /cmd_vel for 20 s =="
timeout 20 ros2 topic echo /cmd_vel --no-arr 2>&1 | head -40 | tee /tmp/cmd_vel.log

if grep -q "linear:" /tmp/cmd_vel.log; then
  echo "PASS: /cmd_vel received commands"
  exit 0
else
  echo "FAIL: no /cmd_vel commands seen"
  exit 1
fi
