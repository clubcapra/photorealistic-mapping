#!/bin/bash
# Sim + manual /cmd_vel drive + bag recording for SLAM-vs-GT map accuracy.
# No SLAM in the live graph — runs offline on the recorded .rec to avoid
# TF tree conflicts with sim's own /tf publishers. Outputs:
#
#   sim.log                Webots driver + supervisor
#   drive.log              /cmd_vel publisher
#   correlation_bag/       /livox/lidar + /ground_truth/odom + /tf_static
#                           + /odom (sim wheel) + /tf (sim)

set +u
cd /home/iliana/prog/photorealistic-mapping
source /opt/ros/humble/setup.bash 2>/dev/null
source install/setup.bash 2>/dev/null
source .claude/worktrees/sim-webots/install/setup.bash 2>/dev/null

# Webots TMPDIR fix
export TMPDIR=/tmp
unset DISPLAY WAYLAND_DISPLAY
export ROS_DOMAIN_ID=133 WEBOTS_GUI=false WEBOTS_PORT=1235 QT_QPA_PLATFORM=xcb

OUT=/tmp/sim_slam_map_test
rm -rf "$OUT" && mkdir -p "$OUT"

# Clean stale Xvfb locks + any leftovers
for lock in /tmp/.X*-lock; do
    [ -f "$lock" ] || continue
    n=$(basename "$lock"); n=${n#.X}; n=${n%-lock}
    pgrep -f "Xvfb :$n " >/dev/null 2>&1 || rm -f "$lock" "/tmp/.X11-unix/X$n" 2>/dev/null
done
pkill -9 -f 'webots|sim_webots|Xvfb|ros2 bag record' 2>/dev/null
sleep 2
rm -rf /tmp/webots/iliana 2>/dev/null

echo "[smoke] STEP 1: launch sim (indoor_office.wbt, fast mode)"
nice -n 10 xvfb-run -a --server-args="-screen 0 1024x768x24" \
    ros2 launch rove_sim_webots sim.launch.py \
        world:=indoor_office.wbt mode:=fast \
    > "$OUT/sim.log" 2>&1 &
SIM_PID=$!

cleanup() {
    [ -n "$REC_PID" ] && kill -SIGINT $REC_PID 2>/dev/null
    [ -n "$DRV_PID" ] && kill -9 $DRV_PID 2>/dev/null
    [ -n "$SIM_PID" ] && kill -SIGINT $SIM_PID 2>/dev/null
    sleep 3
    pkill -9 -f 'webots|sim_webots|Xvfb|ros2 bag record' 2>/dev/null
    return 0
}
trap cleanup EXIT

echo "[smoke] STEP 2: wait for /livox/lidar"
for i in $(seq 1 45); do
    ros2 topic info /livox/lidar 2>/dev/null | grep -q "Publisher count: [1-9]" \
        && { echo "  /livox/lidar live after ${i}s"; break; }
    sleep 1
done
# extra settle so /tf_static + /ground_truth/odom are flowing
sleep 4

echo "[smoke] STEP 3: bag-record /livox/lidar + /ground_truth/odom + tf"
nice -n 10 ros2 bag record -o "$OUT/correlation_bag" \
    /livox/lidar /ground_truth/odom /odom /tf /tf_static \
    > "$OUT/rec.log" 2>&1 &
REC_PID=$!
sleep 2

echo "[smoke] STEP 4: drive trajectory (spin 360 deg then translate 0.5 m)"
# Use a small Python driver to publish cmd_vel
python3 - <<'PYEOF' > "$OUT/drive.log" 2>&1 &
import rclpy
from geometry_msgs.msg import Twist
import time, math

rclpy.init()
node = rclpy.create_node('drive_for_map_test')
pub = node.create_publisher(Twist, '/cmd_vel', 10)

# Wait for subscribers
deadline = time.time() + 8.0
while time.time() < deadline:
    if pub.get_subscription_count() >= 1:
        break
    time.sleep(0.1)
print(f'subscribers: {pub.get_subscription_count()}', flush=True)

def publish_for(lx, az, duration_s, hz=20):
    t = Twist(); t.linear.x = lx; t.angular.z = az
    end = time.time() + duration_s
    while time.time() < end:
        pub.publish(t)
        time.sleep(1.0 / hz)

# Phase 1: spin in place at 0.4 rad/s — 1 full rotation in ~16 s
print('phase 1: spin', flush=True)
publish_for(0.0, 0.4, 18.0)
# Phase 2: forward 0.5 m at 0.15 m/s ~ 3.5 s
print('phase 2: forward', flush=True)
publish_for(0.15, 0.0, 4.0)
# Phase 3: hold still
print('phase 3: stop', flush=True)
publish_for(0.0, 0.0, 1.0)
print('done', flush=True)
node.destroy_node()
rclpy.shutdown()
PYEOF
DRV_PID=$!
wait $DRV_PID
echo "[smoke] STEP 5: drive finished, recording extra 2 s"
sleep 2

echo "[smoke] STEP 6: stop recorder"
kill -SIGINT $REC_PID 2>/dev/null
sleep 3

echo "[smoke] DONE — bag:"
ros2 bag info "$OUT/correlation_bag" 2>&1 | grep -E 'Duration|Messages|Topic'
