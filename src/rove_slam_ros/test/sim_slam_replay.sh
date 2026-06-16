#!/bin/bash
exec 2>&1
cd /home/iliana/prog/photorealistic-mapping
source /opt/ros/humble/setup.bash 2>/dev/null
source install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=133

OUT=/tmp/sim_slam_map_test
SLAMOUT="$OUT/slam_replay"
mkdir -p "$SLAMOUT"

pkill -9 -f "rove_slam_node|rosbag2_player|ros2 bag" 2>/dev/null
sleep 2

echo "[smoke] STEP 1: launch SLAM (wall-time)"
nice -n 10 ros2 launch rove_slam_ros slam.launch.py \
    use_sim_time:=false \
    > "$SLAMOUT/slam.log" 2>&1 &
SLAM_PID=$!

for i in $(seq 1 30); do
    ros2 topic info /livox/lidar 2>/dev/null | grep -q "Subscription count: [1-9]" \
        && { echo "  SLAM subscribed after ${i}s"; break; }
    sleep 1
done

echo "[smoke] STEP 2: record SLAM outputs"
nice -n 10 ros2 bag record -o "$SLAMOUT/slam_bag" \
    /odom /tf /tf_static \
    > "$SLAMOUT/rec.log" 2>&1 &
REC_PID=$!
sleep 1

echo "[smoke] STEP 3: replay sim bag (remap /tf + /tf_static to dead-ends)"
ros2 bag play /tmp/sim_slam_map_test/correlation_bag \
    --remap /tf:=/tf_bag_unused /tf_static:=/tf_static_bag_unused \
    > "$SLAMOUT/play.log" 2>&1 &
PLAY_PID=$!

wait $PLAY_PID
echo "[smoke] STEP 4: bag done"
sleep 2

echo "[smoke] STEP 5: stop"
kill -SIGINT $REC_PID 2>/dev/null
sleep 3
kill -SIGINT $SLAM_PID 2>/dev/null
sleep 3
pkill -9 -f "rove_slam_node|ros2 bag" 2>/dev/null

echo "[smoke] DONE"
ros2 bag info "$SLAMOUT/slam_bag" 2>&1 | grep -E "Duration|Topic|Messages"
