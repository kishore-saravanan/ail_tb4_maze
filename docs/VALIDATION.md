# Validation summary

The tested platform is Ubuntu 24.04, ROS 2 Jazzy and Gazebo Harmonic with the official TurtleBot 4 standard model. The current baseline builds its own map from LiDAR and odometry, then routes to reachable frontiers with early retirement of observed targets.

## Automated checks

The package suite passed **94 test entries** before the latest whitespace-only cleanup. Coverage includes world geometry, scoring, unlimited timers, reachable frontier selection, route clearance, observed-goal retirement, path following and Pledge comparison behavior. The return-spacing cleanup preserves Python syntax trees; Ruff and the spacing checker pass.

Run the checks from a sourced, built workspace:

```bash
colcon test --packages-select ail_tb4_maze
colcon test-result --verbose
python3 scripts/check_return_spacing.py
```

## Visible simulation samples

| Maze side | Grid | Seed | Simulated time | Distance | Contacts | Result |
| --- | --- | --- | ---: | ---: | ---: | --- |
| 12 m | 6×6 | 198382678 | 91.85 s | 20.94 m | 0 | Escaped |
| 12 m | 8×8 | 109460176 | 75.73 s | 16.47 m | 0 | Escaped |
| 16 m | 8×8 | 1773841193 | 199.61 s | 49.29 m | 0 | Escaped |
| 16 m | 10×10 | 1631833898 | 152.38 s | 34.38 m | 0 | Escaped |
| 12 m | 6×6 | 7 | 126.38 s | 26.77 m | 0 | Escaped |
| 20 m | 12×12 | 42 | 589.21 s | 144.02 m | 0 | Escaped |

The first four samples requested RTF 2 with a 300 simulated-second budget. The seed-7 recording requested RTF 1 with the same budget. The larger seed-42 recording requested RTF 2 with no deadline. All used the 0.30 m/s cap; these are development samples, not a controlled reliability study. The [featured recording](media/complex-maze-ending.mp4) shows the end of the larger run.

## Limits and interpretation

- One sample per listed configuration does not establish a success rate, optimality or performance on arbitrary mazes.
- The generator keeps the exit on the east edge and randomizes its row. The solver is not told that location.
- SLAM drift and simulation scheduling can change trajectories and measured time. Loop closure is disabled in the shipped SLAM configuration because repetitive corridors caused false matches in development.
- The independent referee uses privileged simulation poses and contacts for scoring; that information is not passed to the solver. Successful escape does not require zero contacts.
- The baseline can backtrack and has no persistent corridor graph. See [algorithm details and limitations](ALGORITHM.md).

Detailed historical reports, source hashes, raw trajectories and capture metadata are retained locally under the ignored runtime and documentation paths. New trial evidence is written to `.runtime/results/experiment_*/`.
