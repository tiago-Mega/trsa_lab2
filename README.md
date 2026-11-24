# TRSA Lab 2 - Student Starter (ROS 2 Jazzy / Gazebo Harmonic)

Author/Contact: Prof. Ricardo Silva Peres (`ra.peres@fct.unl.pt`)

This is a **template** project. The simulator
(`trsa_warehouse`) is provided; the `trsa_lab2` launch files and four parameter
YAMLs are to be implemented as part of the student's work. 

## Quick setup (Ubuntu 24.04 / WSL2 Bash)

Keep the extracted workspace inside the Linux home directory when using WSL2.
From the directory containing `ros2_ws`, with ROS 2 Jazzy installed:

```bash
source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep \
  ros-jazzy-ros-gz ros-jazzy-nav2-bringup ros-jazzy-navigation2 \
  ros-jazzy-nav2-collision-monitor ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization ros-jazzy-teleop-twist-keyboard
# If rosdep has not been initialised yet: sudo rosdep init
rosdep update --rosdistro jazzy
rosdep install --from-paths ros2_ws/src --ignore-src --rosdistro jazzy -y
python3 tools/fetch_assets.py --world all --robot holonomic \
  --output ros2_ws/src/trsa_warehouse/assets
cd ros2_ws
colcon build --symlink-install --parallel-workers 1
source install/local_setup.bash
ros2 run trsa_warehouse validate_model
ros2 launch trsa_warehouse simulation.launch.py
```

Repeat the ROS environment `source` commands in each new terminal (or add them to `~/.bashrc`) Do not
launch a second simulator while the first one is running. Unfinished student
launch files deliberately raise `NotImplementedError`.

## Responsibilities

- `trsa_warehouse`: Gazebo robot/world, ROS–Gazebo bridge, wheel odometry,
  laser and IMU, joint states, model/mesh fix and LiDAR exclusion. Its
  `config/sensors.yaml` is provided, **not** student configuration work.
- `trsa_lab2`: Implement SLAM, AMCL, EKF, Nav2 parameter configurations;
  mapping/localisation/navigation/fusion/bringup launch files; four-waypoint
  `NavigateToPose` client and experiment handling.
- `trsa_lab2/config/{slam,amcl,ekf,nav2}.yaml` deliberately contain **only
  official Jazzy documentation links**. You must write their parameters yourself.

## Direct command routing

```text
Nav2 controller (cmd_vel_nav) -> velocity smoother (cmd_vel_smoothed)
  -> Nav2 Collision Monitor (/cmd_vel, geometry_msgs/msg/Twist)
  -> ROS–Gazebo bridge -> Gazebo DiffDrive
```

Configure Nav2 Collision Monitor as the **last autonomous command publisher**.
The provided ROS–Gazebo bridge subscribes to `/cmd_vel` as an unstamped `Twist`;
for Jazzy use `enable_stamped_cmd_vel: false` across velocity-processing nodes.
Inspect the velocity smoother and Collision Monitor topic connections; do not
publish from the smoother directly to `/cmd_vel` while Collision Monitor is in
use. The installed Jazzy Nav2 launch has normal remappings for this chain.

**Manual teleoperation:** the normal `/cmd_vel` keyboard command reaches
Gazebo **directly**, bypassing Nav2 Collision Monitor. Never run keyboard
teleop and autonomous navigation concurrently. The Nav2 Collision Monitor is an obstacle-response layer, not
an emergency stop or a substitute for correct action cancellation.

Stop an autonomous run by **cancelling the active NavigateToPose goal** and
verifying the terminal result, then publish zero velocity if necessary:

```bash
ros2 service call /navigate_to_pose/_action/cancel_goal \
  action_msgs/srv/CancelGoal '{}'
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
```

Do not rely on a single zero-velocity message if Nav2 remains active and
continues commanding motion. Inspect `/cmd_vel` and `/odom` to confirm stopping.
Restart the simulation for a known initial pose after intervention.

## Fixed ROS interfaces

| Purpose | Topic / frame |
|---|---|
| Laser | `/scan` (`laser_frame`) |
| Wheel odometry | `/wheel/odom` |
| IMU for EKF | `/imu/data` (`imu_link`) |
| Selected local odometry | `/odom`, `odom -> base_footprint` |
| World/map alignment | `map -> odom` (SLAM **or** AMCL, never both) |
| Nav2 final velocity | `/cmd_vel` (`geometry_msgs/msg/Twist`) |
| Gazebo raw sensor streams | `/imu`, `/wheel/odom_raw` |
| Navigation action | `/navigate_to_pose` (`nav2_msgs/action/NavigateToPose`) |
| Low-level lift input | `/lift/command` (Float64, metres), direct bridge |

Only **one** local TF publisher: `wheel_odom_interface` while the wheel is
selected, or your EKF in active fused mode. The conditioned IMU has no valid
absolute orientation. Ground truth is for evaluation only.

## Four blank YAMLs and the installed Jazzy defaults

Your four student YAMLs include only links to official documentation. The
installed Nav2 example is at:

```bash
less "$(ros2 pkg prefix --share nav2_bringup)/params/nav2_params.yaml"
```

The installed stock example illustrates Nav2's configuration, but
it is **not** specific to this warehouse. Students must select the correct
frames, robot footprint, scanner, global planner (NavFn A* vs Dijkstra),
controller limits and costmaps; set all needed runtime parameters including
optional Nav2 nodes that their chosen navigation launch starts.

When adapting the installed Nav2 file, inspect filters and associated servers:
`KEEPOUT_ZONE_ENABLED` and `SPEED_ZONE_ENABLED` are upstream substitution
placeholders, not actual Boolean values. Do not enable keepout/speed filters
without supplying their mask publishers or disabling those filters.

Key docs:
- Nav2 Jazzy setup: https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/
- Configuration guide: https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/
- Collision Monitor: https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/collision_monitor/configuring_collision_monitor_node/

## Route assignment

`trsa_lab2/route_client.py` contains a self-contained skeleton: students
must implement goal creation, asynchronous action result handling, sequencing,
cancellation and zero-command stopping. The provided CLI/YAML validation and
JSON output format are not a completed navigation implementation.

Prepare a route using four real free-space poses in your own SLAM map.

The map frame is `map`, positions are metres and yaw is radians. Route
success requires a completed action result, not merely goal acceptance.