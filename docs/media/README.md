# Project media

- [Setup GUI](setup.png): actual full scrollable control panel.
- [Strategy diagram](strategy.svg): illustrative explanation of the baseline.
- [Maze ending clip](complex-maze-ending.mp4): actual visible Gazebo and RViz recording, cropped, trimmed and labelled.

The featured clip shows the end of a random **20 m × 20 m, 12 × 12 cell** maze, seed **42**, using the `direct` backend and an unlimited timer. The independent referee recorded **589.213 simulated seconds**, **144.018 m** and **zero contact episodes**, ending in full-footprint escape.

The approximately 28-second clip plays at **1× screen playback**. Simulation real-time factor targeted **2×**, and the robot's speed cap stayed at **0.30 m/s**. Requested simulation speed is not guaranteed wall-time synchronization. No robot motion, map geometry or sensor returns were synthesized. Gazebo's world view is for the operator; the solver receives no generated map or exit coordinates.

Raw recordings, detailed results, earlier demos and LinkedIn edits remain local and are excluded from Git.
