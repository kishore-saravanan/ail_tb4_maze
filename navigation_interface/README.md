# TurtleBot 4 maze controller

Press **Start timer**, then **Start solver** in the setup GUI. The default `controller.py` starts `direct_navigation.launch.py`, containing live-map frontier routing and SLAM Toolbox. **Stop solver** stops that process group. Reset the trial before each fresh attempt.

The controller has no preloaded maze, dimensions, cell spacing or exit coordinates. It receives `/scan`, `/odom`, allowed TF and the simulation clock in navigation domain 62. SLAM derives `/map` from those sensors. `/cmd_vel` uses `TwistStamped`; the gateway independently clamps linear speed to 0.30 m/s and angular speed to 1.50 rad/s and only permits motion during an active trial.

## Default: live-map frontier routing

`direct_navigation.py` plans through observed free space to a reachable frontier selected by `frontier_planner.py`. It uses shared map connectivity, clearance and visit history rather than a separate wall-following pass for each side of a corridor. `path_follower.py` follows the simplified route and checks its swept circular footprint against actual LiDAR.

The map is checked every 0.5 simulated seconds. `frontier_goal_active` retires an old goal when its connected local free-space region no longer borders unknown space. This lets a scan finish observing a dead end without requiring the robot to reach its old goal first. A frontier separated from the goal by a wall does not count. Route changes trigger a new search through the current map; the robot can still return along a known corridor to reach an unexplored branch.

Telemetry includes `route_revision`, `observed_goal_replans`, `replan_reason` and `speed_cap_mps`, plus mapped pose, sensor maps, commands and planned route. `/plan` is green in RViz; `/travelled_path` is the actual cyan trail. This is an occupancy-grid planner, not a persistent corridor graph or a claim of zero repeated travel.

## Comparison: Pledge and dead-end avoidance

- `ail_tb4_maze/pledge.py`: preferred-direction travel, left-wall junction decisions, committed turns, an unwrapped signed SLAM heading, swept-footprint checks and bounded reverse recovery.
- `ail_tb4_maze/dead_end.py`: footprint-aware sensor-grid proof and closed-corridor memory. Thin observed wall surfaces are inflated by the robot clearance radius. A robot-sized unknown continuation, another entrance, sideways branching or a map edge invalidates a proof. Each closed branch is one entry-to-terminal record; every stored record is rechecked against the current map before steering uses it.
- `ail_tb4_maze/pledge_navigation.py`: ROS sensor integration, stale-data stop, telemetry and RViz markers.
- `config/slam.yaml`: 5 cm sensor-built map. Loop closure is disabled because repetitive corridors caused false corrections during development; accumulated drift remains a limitation.

Motion and signed rotation use the LiDAR-corrected SLAM pose; raw wheel odometry is logged separately. Repeated turns produced substantial wheel slip in the simulation, making wheel odometry alone unsuitable for steering. SLAM heading corrections can still affect the turn estimate.

The motion controller assumes orthogonal corridors aligned with the initial heading. It discovers junctions directly from LiDAR, without assuming a cell width. Clearance and collision prediction always use real LiDAR, even when a virtual doorway line is used to suppress a proven dead-end turn. The real-time proof handles a subset of the ideal 2D algorithm's closed branches; physical results must be evaluated separately.

`/explorer/status` and `exploration_*.jsonl` include action, signed rotation, closed/skipped corridor records, proof diagnostics, recovery count, mapped/odometry pose and commands. A corridor record can be invalidated and rediscovered as SLAM changes; counts are not a general map-completion measure. `/pledge/debug` publishes RViz text, red entry-to-terminal edges for closed corridors and a red line for a currently skipped doorway. **Navigation map** opens the sensor-side view; the setup's 2D preview remains the operator's full generated layout.

## Launch and comparisons

Source ROS and the workspace, start the setup timer, then use one controller:

```bash
ros2 launch ail_tb4_maze solver.launch.py navigation_backend:=direct
```

`navigation_backend:=direct` is the default, including when invoked by the GUI. `navigation_backend:=pledge` starts the wall-following comparison, and `navigation_backend:=nav2` starts the earlier frontier/Nav2 stack. Do not run two controllers in the same domain.

The older reactive right-hand baseline is `navigation_interface/reactive_controller.py`; it remains separate from Pledge. Its recorded timeout results remain in the validation notes. None of these implementations guarantees the fastest route through an unknown maze or completion within 300 seconds.

Upstream external dependencies and comparison references:

- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Nav2 Regulated Pure Pursuit](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/configuring_regulated_pp/)
- [m-explore-ros2](https://github.com/robo-friends/m-explore-ros2)

No external maze-solving source code is vendored.

## Robot model and travelled trail

**Navigation map** launches `navigation_view.launch.py`. It displays official TB4 visual meshes using nominal fixed joint positions, anchored to sensor-side `base_link` TF. `/travelled_path` is a cyan trail of SLAM poses; it does not use privileged simulator pose. `navigation_view.py` provides the latched trail and `/display/clear_path` service. `display_model.py` removes simulator/control tags from the visual URDF and gives its fixed display frames separate names, so they do not replace navigation transforms. The perspective camera follows the robot; optional status text is under the `pledge_status` marker namespace.

The Pledge comparison selects an observed open corridor at startup before entering wall-following. Its heading becomes the initial preferred direction once, after entry. Unwrapped rotation is retained, and later wall-following never clears a full-revolution history. This avoids the observed start-room circle but can change the route; it is not a guaranteed speed improvement.
