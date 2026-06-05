#!/usr/bin/env bash
# One-stop SLAM-vs-GT accuracy test in Webots sim. Runs three phases:
#   1. sim_slam_record.sh — bring up sim headless, drive trajectory,
#      record /livox/lidar + /ground_truth/odom + /tf*
#   2. sim_slam_replay.sh — replay the bag through rove_slam_node,
#      record SLAM's /odom
#   3. scripts/sim_slam_accuracy.py — compute pose error + map-cloud NN
#      distances, write report
#
# Total wall-clock: ~70 s. Outputs under /tmp/sim_slam_map_test/.

set +u
WS="$(cd "$(dirname "$0")/../../.." && pwd)"
TEST="$WS/src/rove_slam_ros/test"
SCR="$WS/src/rove_slam_ros/scripts"

echo "[acc] STEP 1 — record sim bag"
bash "$TEST/sim_slam_record.sh" || { echo "record failed"; exit 1; }

echo
echo "[acc] STEP 2 — replay through SLAM, record /odom"
bash "$TEST/sim_slam_replay.sh" || { echo "replay failed"; exit 1; }

echo
echo "[acc] STEP 3 — analyze"
bash -c "source /opt/ros/humble/setup.bash 2>/dev/null && python3 '$SCR/sim_slam_accuracy.py'"
