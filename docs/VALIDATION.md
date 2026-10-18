# Executed validation and limitations

Validation performed on this machine using Ubuntu 24.04.5 LTS, ROS 2 Jazzy and Gazebo Harmonic / Sim 8.15.0. Exact reused ROS package versions are retained in `installed_packages.txt`. Infrastructure checks below are separate from the autonomous wall-following experiments.

| Requirement | Executed evidence | Outcome |
|---|---|---|
| Workspace build | `scripts/build.sh`, ament CMake / colcon | Passed |
| Official TurtleBot 4 in modified official maze | Live headless simulator trials and visible Gazebo launch | Passed; standard TurtleBot 4, original start inside maze |
| LiDAR | Live controller-domain subscriptions | 640 finite ranges at the start; valid message frame and full-circle scan |
| Wheel odometry | Live subscriptions plus straight velocity test | Valid odom/base_link frames; starts near local zero despite nonzero world spawn; changes with wheel motion |
| IMU | Live subscriptions | Gravity acceleration approximately 9.7999 m/s² and valid orientation |
| TF | Late-subscriber checks on both dynamic and latched static TF | All allowed base/LiDAR/IMU transform edges present; no world transform in controller domain |
| Velocity moves robot | Runtime collision and exit cases | Passed through controller-domain `/cmd_vel` and the gateway |
| Collision-free initial pose | Live wall contact subscriptions before starting each trial and geometry checks | No initial wall contacts; static start clearance 1.5 m |
| Physical maze traversal feasibility | Footprint-inflated static clearance / connectivity test | Reachable exit; 0.1 m grid, 0.23 m radius plus 0.02 m margin; original loop has 0.5 m clearance |
| Physical exit crossing and success referee | `.runtime/results/runtime_success/validation.json` | Passed; drove from test spawn near exit through opening; success after 4.974 simulated seconds, 1.441746 m distance, zero collisions |
| Real wall collision scoring | `.runtime/results/runtime_collision/validation.json` | Passed; one collision episode; 1.340142 m ground-truth travel; operator abort saved failure after 9.172 simulated seconds |
| Full 300-second timeout | `.runtime/results/runtime_timeout/validation.json` | Passed; timeout at 300.009 simulated seconds, zero collisions |
| Simulation pause | Same timeout trial | Two wall-clock seconds paused, zero simulation-time advancement |
| Machine-readable scores | JSON files for three live cases | Passed; schema/status/reasons/elapsed/distance/collision fields checked |
| Intended controller-interface isolation | Live topic/service discovery on ROS domain 62 | Only seven permitted interface topics plus ordinary ROS metadata; no privileged topics or services |
| Scoring and world generation | 9 scoring and 7 world-generation pytest cases via colcon | Passed; 16 pytest cases, 0 failures |
| Live setup controls | Randomize, size/grid changes, preview synchronization, timer start/stop, failure/result display and reset | Passed; same simulator launch PID throughout |
| Controller Start/Kill | GUI controller process management | Process group starts and stops in domain 62 |
| GUI visibility | Operator screenshots `gazebo_verified.png`, `rviz_verified.png` | Gazebo maze and RViz robot/LiDAR visible; launch opens both by default |

Validation logs are generated locally and excluded from Git. Per-case evidence is stored under `.runtime/results/runtime_collision/`, `.runtime/results/runtime_success/`, and `.runtime/results/runtime_timeout/`. `.runtime/results/geometry.json` records geometry statistics and the adapted world's checksum. `.runtime/results/interface_probe.json` is the earlier detailed late-subscriber interface inventory. Each runtime trial was isolated by restarting the infrastructure.

The collision trial's wheel odometry advanced further than the physical robot after wall contact. This is expected encoder integration while wheels slip, and confirms the controller is not being given the referee's ground-truth distance as odometry.

## Conditions tested only in deterministic unit tests

Full-footprint versus partial crossing, leaving outside the aperture, starting outside, simulation-time reversal, per-wall collision debouncing, terminal-state immutability, returning inside after an incomplete crossing, and a lifted/fallen pose were injected directly into the pure scoring state machine. These were not all reproduced as physical simulator faults. Ground-truth stream-loss handling and forced infrastructure shutdown during an active trial have code paths but were not deliberately fault-injected in the live simulator.

## Known limitations

- **Logical isolation only.** A deliberately noncompliant same-user process can access infrastructure files, switch ROS domains or use Gazebo Transport. ROS domains separate traffic logically; filesystem/network/DDS access is not cryptographically enforced. The intended ROS interface was tested; hostile access prevention was not.
- The official maze includes wide open regions. It has branching routes, dead ends and loops, but is not a uniformly narrow competition Micromouse maze. Wall following may loop around disconnected obstacles or exceed the time budget.
- Whole-footprint success uses a conservative circular 0.23 m radius approximation rather than exact oriented mesh intersection. This prevents premature scoring at the opening and may require slightly more outward travel.
- Contact counts are contact **episodes**, not impulsive force measurements. Multiple distinct wall links can count separately; long continuous contact is one event per wall. The live physical collision case verified counting, not every possible multi-wall impact configuration.
- Distance is sampled planar ground-truth travel, so microscopic physics jitter may accumulate and fast motion between samples can be slightly underestimated. Nominal model-pose publication is 62 Hz.
- IMU and LiDAR inherit the official idealized simulation sensor definitions. No additional calibrated real-world noise was introduced.
- Operator RViz displays passive wheel suspension joints using default values merged with live encoder wheel angles. These extra visual transforms are filtered out of the controller interface. The robot physics itself retains the original suspension model.
- Gazebo emits EGL/NVIDIA driver warnings on this system, but sensor data and the visible rendering were verified. No graphics drivers or system configuration were changed. The wheel controller logs normal braking warnings while no controller is publishing commands.
- Trial results and controller telemetry are saved locally. The optional renderer exports a LiDAR/decision replay; record Gazebo separately for the simulation footage.
- The gateway blocks commands after trial completion. The experiment helper also stops the solver process; the GUI provides Stop solver.
- Full live tests need an unused challenge ROS domain / Gazebo partition. Do not start multiple instances using this fixed 61/62-domain configuration simultaneously.

The environment is ready for the **supervised navigation challenge using the documented interface**, with the isolation limitation above. It is not an adversarially sandboxed competition service.

## Autonomous wall-following experiment — 2026-10-18

The final controller passed 10 wall-following tests, alongside 9 scoring and 7 world-generation tests (26 pytest cases; colcon includes 3 additional CTest entries). The GUI Start/Stop process handlers and the MP4 renderer were exercised. The local source hashes and reproducible parameters are retained in [experiment_results.json](experiment_results.json).

| Layout | Maze | Outcome | Simulated time | Ground-truth distance | Recorded collision episodes |
| --- | --- | --- | --- | --- | --- |
| Official | 20 × 20 m | Timeout | 300.005 s | 45.83 m | 0 |
| Random, seed 7 | 12 × 12 m, 6 × 6 cells | Timeout | 300.015 s | 34.58 m | 0 |

These were headless regression trials with target real-time factor 3; the time budget remained 300 simulated seconds. Both mazes were used during development. This is not a blind evaluation, a general success rate, or proof of collision avoidance. The controller made slow/stalled progress at some corners and used local reverse recovery. **It did not escape these two trials.**

The experiment short should show the actual timeout, or record a new separately specified trial and report its observed outcome. Do not infer that every wall-following implementation would fail these mazes: this result combines the chosen rule, this controller's tuning and the simulator dynamics.
