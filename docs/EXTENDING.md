# Run your own navigation algorithm

The lab separates the **navigation algorithm** from maze generation, visualization and scoring. You can change one decision rule or replace the controller completely.

## Option A: change a part of the included solver

Keep the default `navigation_interface/controller.py` entry point. It launches SLAM and the `direct` baseline together.

- To test a different frontier heuristic, edit `choose_frontier()` in `ail_tb4_maze/frontier_planner.py`. With `return_path=True`, return `(target, frontier_centers, route)`, using map-frame coordinates; return `(None, centers, [])` when no reachable frontier exists.
- To change early goal retirement, edit `frontier_goal_active()` and add cases covering newly observed corridors, walls and map edges.
- To test another motion policy, edit `PathFollower.step()` in `ail_tb4_maze/path_follower.py`. Preserve stale-data stops and validate your collision checks.

Use the same seeds and time budgets before and after a change. Run `colcon test --packages-select ail_tb4_maze` and inspect `colcon test-result --verbose`.

## Option B: replace the controller

Copy [`examples/custom_controller.py`](../examples/custom_controller.py), implement its `decide_velocity()` method, and select the copy's path in the GUI's **Navigation controller** field. The provided example deliberately publishes zero velocity. It is a starting interface, not a maze-solving policy.

1. Reset the robot/trial and choose a layout.
2. Set the time limit before starting the attempt. Zero means unlimited.
3. Press **Start timer**, then **Start solver**. The GUI launches your Python script in navigation domain 62, with `use_sim_time:=true` and a `recording_path` parameter override.
4. **Stop solver** terminates its process group. Reset before a fresh attempt.

A source-checkout path may be relative to the GUI's working directory; use an absolute path when launching from another directory. For an installed package, the example is under its `share/ail_tb4_maze/examples/` directory.

### Interface

| Direction | Topic | Type | Notes |
| --- | --- | --- | --- |
| Input | `/scan` | `sensor_msgs/msg/LaserScan` | Sensor-data QoS; scan frame and mounting transform matter |
| Input | `/odom` | `nav_msgs/msg/Odometry` | Wheel odometry rebased to the start; not simulator ground truth |
| Input | `/imu` | `sensor_msgs/msg/Imu` | Robot IMU |
| Input | `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Allowed robot/sensor transforms |
| Input | `/clock` | `rosgraph_msgs/msg/Clock` | Use simulation time |
| Output | `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | Stamp with the simulation clock; use `base_link` |

The gateway clamps linear velocity to ±0.30 m/s and angular velocity to ±1.50 rad/s, drops nonfinite commands, stops stale commands and permits motion only during an active trial. **The baseline's swept-footprint collision guard is part of the baseline controller. A full replacement must implement its own collision avoidance.**

The default infrastructure/navigation domains are 61/62. The reproducible experiment helper uses 63/64 by default; obtain domain choices from its arguments rather than hardcoding a different domain inside your node. Never run two command-producing controllers in one navigation domain.

### Mapping is your choice

Selecting a replacement script replaces the default navigation manager; it does not automatically start SLAM for that script. Use LiDAR directly, start your own mapper, or launch the supplied SLAM configuration separately:

```bash
# Source ROS and the workspace first. For the GUI's default navigation domain:
ROS_DOMAIN_ID=62 ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:="$(ros2 pkg prefix --share ail_tb4_maze)/config/slam.yaml" \
  use_sim_time:=true autostart:=true
```

This produces `/map` and map-relative TF from the allowed sensors. Do not launch an extra SLAM instance if the default solver is already running. A custom mapper publishing `/map` can be inspected by adding that map topic in RViz; the baseline's fused `/navigation/map` topic is produced only by its own mapping code.

### Fair evaluation

The solver should not read generated world/configuration files, subscribe to privileged pose/contact topics, use the operator's full maze preview as input, or query simulator ground truth. The referee uses that information independently to score the attempt. ROS domains provide logical separation, not an OS sandbox against a deliberately privileged same-user process.

The physical exit is not supplied as a navigation goal. The referee marks a full-footprint outward crossing as success and stops motion. Contact episodes and sampled ground-truth distance are recorded independently. A successful crossing may still have contacts, so report both outcome and contact count.

Keep a fixed seed, size, grid density, speed cap and duration budget for a comparison. Save failed attempts too. Unlimited trials are useful for learning but are not directly comparable to a bounded 300-second result. Simulated time, wall time and edited playback speed are different quantities.

See [ALGORITHM.md](ALGORITHM.md) for the shipped strategy and [VALIDATION.md](VALIDATION.md) for measured scope and limitations.
