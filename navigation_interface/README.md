# LiDAR wall-following maze solver

Press **Start timer**, then **Start solver** in the setup GUI. The default strategy is the right-hand rule:

1. First turn toward the wall on the right, approach it, then turn left to align alongside it.
2. Follow the wall on the right, aiming for 0.45 m clearance from the robot center.
3. If the front is blocked, stop and turn left until there is room.
4. If the right wall disappears, move past its end and turn right into the opening.
5. At a dead end, repeated left turning turns the robot around.

Two LiDAR sectors estimate right-wall distance and angle. A proportional steering controller corrects both; a short swept-footprint prediction slows/stops the robot before obstacles. Scan loss stops motion. The actual LiDAR-to-base transform is applied before sector measurements.

LiDAR controls ongoing wall following. Wheel odometry measures the short acquisition/corner turns and corner-clearance advance and is also recorded for optional visualizations. Simulation time drives the control timer. The algorithm does **not** read a maze map, dimensions, seed, exit coordinates or referee results. It does not learn or train a neural network: it is a classical reactive navigation algorithm.

Wall following is simple, but it is not a universal solver. A robot can loop around a disconnected obstacle, and a winding route may exceed 300 seconds. The guarantee for suitable wall-connected mazes does not imply that every generated maze will finish within this time limit.

## ROS interface

Inputs: `/scan` (LaserScan), `/tf`, `/tf_static`, `/clock`; `/odom` (Odometry) for acquisition/corner movements and recording. Output: `/cmd_vel` (TwistStamped). Diagnostics: `/wall_follower/status` (JSON in String).

The setup GUI supplies ROS domain 62, `use_sim_time:=true`, and a local JSONL recording path. The gateway permits motion only while a trial runs, clamps speeds to 0.30 m/s and 1.50 rad/s, and stops on publisher loss. Default controller targets are lower than those caps.

Parameters: `wall_side:=right` (or `left`), `wall_distance:=0.45`, `recording_path:=/path/to/trial.jsonl`. Every telemetry entry contains state, front/right distances, actual commanded velocity, local odometry pose, LiDAR endpoints and simulated timestamp. No map is required or built for choosing commands.

The packaged node can also be started with `ros2 launch ail_tb4_maze solver.launch.py` after starting the environment and timer.

A progress watchdog attempts a short LiDAR-checked reverse when commanded motion produces no odometry progress. If reverse space is blocked, it tries a recovery turn. This is local recovery, not a learned or map-based planner.
