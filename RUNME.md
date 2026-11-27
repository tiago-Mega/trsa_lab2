# RUNME.md - TRSA Lab 2 (SLAM, Sensor Fusion and Navigation)

Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic. Work in the Linux filesystem (not /mnt/c).
`trsa_warehouse` is provided and must remain unchanged.

## 1. Layout

```text
trsa_lab2/
|-- ros2_ws/src/
|   |-- trsa_warehouse/            # provided, unchanged
|   `-- trsa_lab2/                 # student package (config/, launch/, trsa_lab2/route_client.py)
|-- tools/fetch_assets.py
|-- data/maps/                          # saved SLAM map (.yaml + image)
|-- data/
|   |--route.yaml                # four map-frame waypoints (x, y, yaw)
|   |--results/<trial>/               # one directory per trial
|      |-- nav2.yaml  ekf.yaml(if used)  route.yaml
|      |-- result.json                # route_client action result
|      |-- bag/                       # rosbag
`-- RUNME.md
```

## 2. Environment (run in EVERY terminal) or add to ~/.bashrc

```bash

export TRSA_ROOT="$HOME/trsa_lab2"
export TRSA_WS="$TRSA_ROOT/ros2_ws"

export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

source /opt/ros/jazzy/setup.bash

cd "$TRSA_WS" || return
source "$TRSA_WS/install/local_setup.bash" 
```

Keep domain, Gazebo partition and workspace identical in all terminals. Never run two simulators.

## 3. One-time install and build

```bash
source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install -y build-essential cmake unzip git python3-colcon-common-extensions python3-rosdep \
  ros-jazzy-ros-gz ros-jazzy-nav2-bringup ros-jazzy-navigation2 ros-jazzy-nav2-collision-monitor \
  ros-jazzy-slam-toolbox ros-jazzy-robot-localization ros-jazzy-teleop-twist-keyboard ros-jazzy-ros2bag
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
rosdep update --rosdistro jazzy
cd "$TRSA_ROOT"
rosdep install --from-paths "$TRSA_WS/src" --ignore-src --rosdistro jazzy -y
python3 tools/fetch_assets.py --world all --robot holonomic --output "$TRSA_WS/src/trsa_warehouse/assets"
find "$TRSA_WS/src" -path '*/scripts/*' -type f -exec chmod +x {} +
cd "$TRSA_WS" && colcon build --symlink-install --parallel-workers 1
source install/local_setup.bash
ros2 run trsa_warehouse validate_model
```

## 4. Tasks 1-4

Task 1 - simulator and teleop (never together with Nav2):
```bash
# Terminal A
ros2 launch trsa_warehouse simulation.launch.py world:=1 publish_wheel_tf:=true \
  rviz_config:="$(ros2 pkg prefix --share trsa_warehouse)/config/warehouse.rviz"
# Terminal B
ros2 run tf2_tools view_frames
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.20 -p turn:=0.5 -p stamped:=false
```

Task 2 - mapping and saving the map:
```bash
# Terminal A
ros2 launch trsa_lab2 bringup.launch.py mode:=mapping navigation:=false
# Terminal B: drive through the aisles
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.20 -p turn:=0.5 -p stamped:=false
# Terminal C: save
mkdir -p "$TRSA_ROOT/maps"
ros2 run nav2_map_server map_saver_cli -f "$TRSA_ROOT/maps/warehouse"
```

Task 3 - AMCL + Nav2 (wheel-only baseline). Set the 2D Pose Estimate in RViz, same spot every run.
Pick route points with RViz "Publish Point" and `ros2 topic echo /clicked_point`, then write `data/route.yaml`
(`frame_id: map`, waypoints with `x`, `y`, `yaw` in metres/radians; at least four).
```bash
# Terminal A
ros2 launch trsa_lab2 bringup.launch.py mode:=localization map:="$TRSA_ROOT/data/maps/warehouse.yaml" \
  use_ekf:=false odometry_source:=wheel navigation:=true
# Terminal D
cd "$TRSA_ROOT"
ros2 run trsa_lab2 route_client <ARGS>   # TODO: match your argparse (route path, output JSON, per-goal timeout)
```

Task 4 - odometry modes (only one local TF publisher; do not switch during a goal):

| Mode | Launch args | Local odometry |
|---|---|---|
| Wheel-only | `use_ekf:=false odometry_source:=wheel` | wheel `/odom` + TF |
| Observer EKF | `use_ekf:=true odometry_source:=wheel` | wheel `/odom`; EKF -> `/odometry/filtered`, no TF |
| Active EKF | `use_ekf:=true odometry_source:=ekf` | EKF -> `/odom` and `odom->base_footprint` |

## 5. Task 5 experiments

Common rules: same map, same four-waypoint route, same initial pose, restart the simulator for each trial.
Per trial: create `results/<trial>/`, copy the exact `nav2.yaml` (and `ekf.yaml` for EKF runs) and `route.yaml`,
start the bag BEFORE the route, stop it with Ctrl+C after the route ends.

Generic trial template (replace `<trial>`):
```bash
T="$TRSA_ROOT/data/results/<trial>"; mkdir -p "$T"
cp "$TRSA_WS/src/trsa_lab2/config/nav2.yaml" "$T/"
cp "$TRSA_ROOT/data/route.yaml" "$T/"
# Terminal A: launch (see section 4 for the mode)
# Terminal C: bag
cd "$T" && ros2 bag record -o bag /scan /odom /cmd_vel /tf /tf_static /plan /amcl_pose /ground_truth/odom
# Terminal D: route
cd "$TRSA_ROOT" && ros2 run trsa_lab2 route_client <ARGS> # output -> $T/result.json
```
After any config edit: `cd "$TRSA_WS" && colcon build --symlink-install --parallel-workers 1`, then restart the launch.

### E1 - NavFn A* vs Dijkstra
Edit `nav2.yaml`: `planner_server.GridBased.use_astar: true|false`. Trials: `E1_astar_false`, `E1_astar_true`.
Record: path, execution time, success/failure.

### E2 - Inflation radius
Edit BOTH `local_costmap` and `global_costmap` `inflation_layer.inflation_radius` (baseline 0.55; suggested 0.25 and 0.85).
Keep `cost_scaling_factor` fixed. Add `/local_costmap/costmap /global_costmap/costmap` to the bag.
```bash
F="$TRSA_WS/src/trsa_lab2/config/nav2.yaml"
sed -i -E 's/(inflation_radius:\s*)[0-9.]+/\10.35/' "$F"; grep -n inflation_radius "$F"
ros2 param get /local_costmap/local_costmap inflation_layer.inflation_radius
ros2 param get /global_costmap/global_costmap inflation_layer.inflation_radius
```
Trials: `E2_radious_small`, `E2_radious_large`. Record: clearance, tight-aisle feasibility, recovery.

### E3 - Wheel-only vs observer vs active EKF
Trials: `E3_wheel_only`, `E3_observer`, `E3_active`. Copy `ekf.yaml` into the EKF trials. Add `/imu/data /wheel/odom /odometry/filtered` to the bag where relevant.
```bash
# Terminal E (EKF runs only)
ros2 run trsa_warehouse fusion_record --ekf-topic /odometry/filtered   # observer
ros2 run trsa_warehouse fusion_record --ekf-topic /odom                # active
```
Ground truth is evaluation-only (never fused). Record: drift, yaw error, navigation behaviour.

### E4 - Failure/timeout and cancellation
Trials: `E4_failure` (unreachable goal in a separate route copy, or very short timeout) and `E4_cancel` (cancel mid-route).
```bash
ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal '{}'
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
ros2 topic echo /odom --field twist.twist.linear
```
Never run keyboard teleop together with Nav2. Verify: terminal action status, zero `/cmd_vel`, robot actually stopped, no further goal sent.
Record bag including `/cmd_vel_nav /cmd_vel_smoothed`.
