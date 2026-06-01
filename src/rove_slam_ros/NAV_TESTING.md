# Headless nav integration — smoke testing

End-to-end smoke procedure for the ROS 2 SLAM + nav2 stack on a recorded
bag. No rviz, no real hardware required.

## Prereqs

- ROS 2 Humble installed at `/opt/ros/humble`
- nav2 stack packages (`apt install ros-humble-nav2-*`)
- A converted rosbag2 directory with `/livox/lidar` and `/imu/data` topics
- This workspace built with `colcon build --packages-up-to rove_slam_ros`

## 1. Build + source

```sh
cd ~/prog/photorealistic-mapping
git submodule update --init --recursive
source /opt/ros/humble/setup.bash
colcon build --packages-select rove_slam_ros --symlink-install
source install/setup.bash
```

## 2. Smoke 1 — SLAM-only, verify topics

```sh
ros2 launch rove_slam_ros slam.launch.py &
ros2 bag play /home/iliana/bags/moving_extra_long_bag2 \
    --remap /tf:=/tf_bag_unused /tf_static:=/tf_static_bag_unused &
sleep 10
ros2 topic hz /odom              # ~10 Hz
ros2 topic hz /cloud_obstacles   # ~2 Hz
ros2 topic hz /tf                # ~20 Hz
ros2 run tf2_ros tf2_echo new_map base_link    # should resolve
```

Expected: TF chain `new_map → odom → base_link` resolves, `/odom`
publishes at the lidar rate (~10 Hz live, faster on accelerated replay).

## 3. Smoke 2 — SLAM + local costmap (lifecycle managed)

```sh
ros2 launch rove_slam_ros bag_replay.launch.py \
    bag:=/home/iliana/bags/moving_extra_long_bag2 \
    with_nav:=true
# wait ~15 s for lifecycle to activate, then:
ros2 topic hz /costmap/costmap   # ~1.7 Hz
```

This is the integration test from `phase-3-3.md` — costmap consumes
`/cloud_obstacles` and publishes its own `/costmap/costmap` rolling
window. If this works, the obstacle layer is correctly wired.

## 4. Headless nav goal test

```sh
ros2 launch rove_slam_ros bag_nav.launch.py \
    bag:=/home/iliana/bags/moving_extra_long_bag2

# In another shell, after ~20 s:
ros2 topic echo /cmd_vel --no-arr &
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose '{
  pose: {
    header: {frame_id: new_map},
    pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}
  }
}'
```

**Pass condition**: `/cmd_vel` publishes non-zero linear or angular
twist within 15 s of the goal send. Recovery behaviors (spin, backup)
count — those also exercise the BT navigator → controller → smoother
pipeline.

### What goes wrong (and what fixed it)

| Symptom                                                         | Cause                                                             | Fix                                                        |
|-----------------------------------------------------------------|-------------------------------------------------------------------|------------------------------------------------------------|
| `Could not find a connection between 'map' and 'base_link'`     | bag's stale `/tf` competes with SLAM live TF                       | bag_nav remaps `/tf` → `/tf_bag_unused` (already in launch) |
| `Sensor origin at (0,0) is out of map bounds`                   | costmap reads sensor origin from cloud's `frame_id` (= `new_map`) | `sensor_frame: base_link` on the obstacle_layer source     |
| Costmap `lifecycle_manager` hangs at "Configuring"              | bond timeout fires before configure completes                      | `bond_timeout: 0.0` in lifecycle_manager params            |
| `Node not recognized: RemovePassedGoals` aborts bringup         | nav2 Humble's default BT references plugins not in default list    | added `nav2_remove_passed_goals_action_bt_node` etc to BT  |
| `Message Filter dropping message ... earlier than transform`    | yaml hardcoded `use_sim_time: false` while launch passed `true`    | stripped `use_sim_time` from yaml (launch-arg flows now)   |
| Two `local_costmap/local_costmap` nodes in `node list`          | spawned `nav2_costmap_2d` AND `controller_server` (embeds costmap) | dropped the standalone costmap Nodes in slam_navfull       |

## 5. Optional — full nav2 stack via nav2_bringup

If `bag_nav.launch.py` has BT or lifecycle issues, the cleanest fallback
is delegating to `nav2_bringup/navigation_launch.py`:

```sh
ros2 launch rove_slam_ros nav_bringup.launch.py
# (separately, replay bag with the /tf remap shown above)
```

This wires our SLAM into nav2's well-tested standard launch tree.

## 6. Stopping

```sh
# Ctrl+C in the launch terminal (sends SIGINT, nav2 shuts down cleanly).
# If something hangs, force:
pkill -9 -f 'rove_slam_node|controller_server|planner_server|bt_navigator|behavior_server|velocity_smoother|lifecycle_manager|nav2_costmap_2d|nav2_bringup|ros2 bag'
```
