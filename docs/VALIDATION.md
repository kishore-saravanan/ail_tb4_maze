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

## SLAM/frontier navigation replacement — 2026-10-18

The default now builds its own map from allowed sensor data, chooses reachable frontiers, and uses Nav2 NavFn A* with Regulated Pure Pursuit. No ground-truth map, dimensions or exit coordinates enter the controller. The previous wall follower remains a comparison script.

The build and all 37 pytest cases passed (41 entries in colcon’s result summary). New checks cover a narrow reachable frontier, an unreachable chamber, failed-goal exclusion, unknown-space shortcuts, construction/cleanup of the explorer and velocity adapter, late action callbacks, and sending a new frontier before the current goal stops the robot, preventing an early reversal at a corner, startup failure and stale-command stopping. The setup window was rendered with the new Navigation map button; a live sensor-built map and `/plan` were captured in `navigation_map.png`.

Development found two failures: a false loop closure in repeated corridors, and stale global scan obstacles that blocked a free passage. The shipped configuration disables loop closure for these short runs and uses the SLAM static layer for global planning, with live obstacle observations in the local costmap. A loop-closure development run timed out at 300.003 s / 20.74 m / 0 contacts; the stale-global-obstacle run was operator-aborted at 166.919 s / 7.95 m / 0 contacts. Exact hashes and parameters are retained in [navigation_results.json](navigation_results.json).

This tuning has not been evaluated on real hardware or a held-out maze set. It is not proof of globally optimal exploration or collision avoidance. The robot’s sensor-built map can drift; disabling loop closure has a cost on longer runs. A single compiled Dijkstra search evaluates frontier travel, but frontier utility remains a heuristic for an unknown exit.

The MPPI controller and MPPI with a rotation shim were also tried as development alternatives, but made poorer progress in the dense maze and were operator-aborted. They are not the shipped controller. Tighter RPP heading/lookahead settings also reduced progress and were reverted. The final default retains the better-performing RPP control parameters and extends frontier targets forward while moving. An unrestricted early handoff was discarded after logs showed that it could reverse away before observing a corner; the shipped version requires a forward direction for early handoff. These are tuning observations, not controlled statistical comparisons.

## Pledge controller replacement — 2026-10-19

The current default is sensor-based Pledge with observed straight dead-end avoidance. The visible Gazebo trial escaped seed 7 (12 m, 6 × 6 cells) in 267.225 simulated seconds / 53.671 m / 0 contacts. There were no dead-end proof events, so this trial does not establish a physical pruning speedup. The final-source rerun also escaped in 267.265 s / 53.760 m / 0 contacts, with matching controller source hashes and no recovery or proof events. The final package suite has 73 passing entries. See [Pledge validation](PLEDGE_VALIDATION.md) and [source-hashed results](pledge_results.json) for exact scope, later refinements and limitations. Earlier sections above describe the historical controllers, not the current default.

## Footprint-aware closed-corridor correction — 2026-10-19

The doorway proof now works with thin LiDAR wall surfaces and robot-centre clearance, rather than expecting filled wall volumes. It compresses each supported closed side branch into an entry-to-terminal record, revalidates memory against map changes, and cancels a pending turn when a proof arrives. The visible seed-7 trial escaped in 201.470 s / 41.124 m / 0 contacts, with three confirmed skips and no scored-run reverse recoveries. The earlier Pledge snapshot took 267.265 s / 53.760 m on this maze. All 79 test entries pass, including 23 Pledge cases. See [the source-hashed corridor result](corridor_results.json) and [scope and limitations](PLEDGE_VALIDATION.md). Open-corridor and loop routing still uses Pledge; this is not a complete topological route planner.


## Repeated corridor traversal fix — 2026-10-19

The default GUI, solver launch, manager and experiment helper now select the live-map `direct` backend. Pledge remains an explicit comparison. This replaces wall-by-wall corridor decisions with shortest routes through observed free space to reachable frontiers. A target is reconsidered every 0.5 simulated seconds: when LiDAR has finished observing its connected local region, the robot abandons that old approach before physically reaching its end. Unknown space behind a wall does not keep a closed target active. The actual travelled trail is retained honestly; required backtracking is not hidden.

All **87 colcon test entries pass** (81 pytest cases and six CTest suites). Four new regressions cover an approach becoming fully observed, unknown space behind a wall, unknown beyond the current map edge, and the ROS controller redirecting toward another branch while still more than four metres from its previous goal. Geometry validation also passes.

One visible Gazebo/RViz run at requested RTF 2, random preset 12 m / 6 cells / seed 7, fully crossed the exit in **103.298 simulated seconds / 24.576 m / zero contacts**. The earlier visible Pledge run on those same parameters took **240.739 s / 50.129 m / zero contacts**. This is 57.1% less simulated time and 51.0% less travel in one development case. Both backend replacement and early goal cancellation contribute; this is not an ablation or a general performance estimate. The corrected run took the side passage near SLAM (1.5, 3.1) before reaching the closed tip near (1.6, 4.5), avoiding the specific perimeter return observed in RViz.

The run recorded 32 observed-goal replans and never commanded more than 0.30 m/s. The controller receives no maze geometry, dimensions or exit. It uses occupancy-grid connectivity rather than a persistent topological corridor graph; it may revisit a corridor when needed to reach another unknown branch. Source hashes, scores and limitations are retained in [live_map_routing_results.json](live_map_routing_results.json).


## Four randomized grid combinations — 2026-10-19

All trials used visible Gazebo and RViz, requested RTF 2, the same recorded solver/configuration hashes, a 300 simulated-second deadline and a 0.30 m/s command cap. Seeds were sampled once before the suite; no failed runs were replaced or discarded.

| Maze side | Grid | Seed | Result | Simulated time | Travel | Contacts |
| --- | --- | --- | --- | --- | --- | --- |
| 12 m | 6 × 6 | 198382678 | success | 91.9 s | 20.9 m | 0 |
| 12 m | 8 × 8 | 109460176 | success | 75.7 s | 16.5 m | 0 |
| 16 m | 8 × 8 | 1773841193 | success | 199.6 s | 49.3 m | 0 |
| 16 m | 10 × 10 | 1631833898 | success | 152.4 s | 34.4 m | 0 |

All four sampled layouts fully crossed the exit, with zero contact episodes. One seed per combination does not establish a reliability rate or a ranking between grid densities. The generator randomizes walls and exit row, while keeping the exit on the east boundary; the planner does not receive that information. Source hashes, individual scores, random seeds and telemetry locations are in [randomized_grid_results.json](randomized_grid_results.json).

A runtime audit in navigation domain 50 found no `/privileged/*` or `/referee/*` topics. The gateway forwards LiDAR, rebased wheel odometry, IMU, permitted TF and clock. The map is built by SLAM; frontier goals are selected on that map. Exit geometry remains in the infrastructure domain for scoring. The independent referee detects complete crossing and stops motion; the solver currently does not recognize and declare escape itself. Domain isolation is logical separation, not an OS sandbox.


## Unlimited trials and navigation RViz — 2026-10-19

The GUI, challenge launch and experiment helper default to `time_limit = 0`, meaning no trial deadline. The GUI exposes a Time limit selector with a No limit value. Its selection is captured by the referee at Start timer and remains fixed for that attempt. Positive deadlines remain supported; use `--time-limit 300` to reproduce the bounded experiments above. The experiment helper's former 900-second wall-clock cutoff is also disabled by default; `--wall-timeout` can enable one explicitly. Results retain elapsed time, distance, contacts and the selected deadline; zero represents unlimited without nonstandard JSON infinity.

All **94 test entries pass** (88 pytest cases and six CTest suites), including a simulated one-hour scoring interval followed by successful crossing, a custom 600-second deadline, invalid limit rejection, clock reversal under unlimited duration, and a ROS referee capturing settings at start/reset while retaining the active attempt's chosen limit. This is injected scoring-time validation, not a physical one-hour Gazebo run. The running visible GUI's referee parameter and status file both reported `time_limit = 0.0`, and its solver was launched through the GUI.

The setup previously opened `challenge.rviz` in the infrastructure domain, where SLAM `/map` was unavailable. Default challenge RViz now includes `navigation_view.launch.py` in a scoped navigation domain; the GUI's Open RViz action also uses that view. The live user's trial was preserved while a sensor-domain viewer was opened and the incorrect viewer closed. Runtime subscription received a 261 × 182 planning map at 0.05 m resolution and a travelled trail; RViz Global Status was OK. A map only appears after the solver starts SLAM.


## Readability and documentation media — 2026-10-22

Public Python modules, launch descriptions, entry scripts, examples and tests now use the pinned Ruff 0.17.1 formatter with four-space indentation, separated imports and an 88-character line limit. Lint includes import sorting, undefined/unused names and PEP 8 error checks. The first formatting-only pass preserved all saved syntax trees. Subsequent cleanup removed unused imports/locals, renamed ambiguous local link variables, expanded the GUI stylesheet and corrected legacy validation helpers to use the package-owned state path and explicit 300-second limits. These helpers now request visible Gazebo; they were not used to manipulate the recorded demo.

All 94 existing test entries pass after cleanup. A fresh visible seed-7 / 12 m / 6-cell / RTF-1 trial escaped in **126.381 simulated seconds / 26.774 m / zero contact episodes**. This is a new run, not the earlier 103.298-second RTF-2 sample; timing/trajectory may vary with simulator scheduling, SLAM and recording load. Its late-run screen excerpt is cropped, trimmed and labeled in [the media guide](media/README.md), with the actual result and source hashes in [demo-capture.json](media/demo-capture.json). No simulator footage was synthesized.

The browser-based 2D testbed and ideal-motion benchmark remain available locally but are Git-ignored and excluded from public documentation/install targets. The staged public package focuses on the ROS/Gazebo lab, algorithm integration and measured results.
