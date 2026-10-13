# Maze selection and attribution

The selected maze is the existing **official TurtleBot 4 simulator maze**. The Jazzy simulator explicitly supports Gazebo Harmonic and lists maze among its available worlds: [official simulator repository](https://github.com/turtlebot/turtlebot4_simulator/tree/jazzy), [official user manual](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html).

Research compared these public sources; only the official world was selected for adaptation. Alternative repositories were inspected for their declared platform, world files and license. They were not executed.

| Candidate | License and compatibility | Geometry / challenge assessment | Adaptation required / decision |
|---|---|---|---|
| [Official TurtleBot 4 maze](https://github.com/turtlebot/turtlebot4_simulator/blob/b7d0f3b973258481f6cd8137bef9240fb567f566/turtlebot4_gz_bringup/worlds/maze.sdf) | Apache-2.0; official Jazzy/Harmonic packages already installed | Measured 20 × 20 m boundary; 36 interior wall links. Original border is completely closed. Box geometry has 1 m passages and wider rooms. Offline checks found 10 dead-end passage cells on a 1 m lattice and a clear circuit around an isolated obstacle. Multiple branching regions. | Split one outer wall to add a 2 m opening; instrument contacts and enable sensor systems. No scaling or interior redesign. **Selected** for compatibility, attribution and minimal changes. |
| [Techfest 2021 International Micromouse Competition template](https://github.com/Maze-Solving-Robot/Micromouse-1) | ROS 1 / Gazebo Classic, with Classic GUI/world plugins. No top-level license found in the inspected tree; reuse permission not established. | Inspected `arena.world`: 2.892 m square base and 0.192 m wall modules with 0.012 m thickness. This scale is unsuitable for a roughly 0.36 m diameter TurtleBot. Detailed connectivity, loops and exterior exit were not validated. | Major rescaling, ROS 2/Harmonic plugin conversion, exit verification and licensing resolution needed. Rejected despite its competition connection. |
| [Micromouse Gazebo DRL](https://github.com/fomalhaut251/micromouse-gazebo-drl) | Top-level MIT license; README specifies ROS 2 Humble. Inspected `td7_static.world` uses SDF 1.6 and `libgazebo_ros_state.so`, a Classic plugin. Some vendored model directories carry separate licenses. | Obstacle training world with included `td7_world` and obstacle assets. Maze topology and an actual escape opening are not established by the inspected world; the included model has boundary-wall spans of 12.15 m and shorter 3.15 m internal sections. Full clearance and topology were not validated. | Classic plugins and assets need porting; topology and scale need further assessment. Greater uncertainty than the installed official maze; rejected. |

Lattice branch counts are descriptive validation statistics, not a claim that each open-room grid cell is a distinct maze junction. The official maze is a branching obstacle maze with relatively open areas, not a uniformly narrow Micromouse maze. Its walls and external escape route make it suitable for this first exploration challenge. Autonomous completion within 300 seconds remains to be evaluated in the later navigation phase.

## Provenance

- Repository: `https://github.com/turtlebot/turtlebot4_simulator`
- Branch: `jazzy`
- Inspected upstream commit: `b7d0f3b973258481f6cd8137bef9240fb567f566`
- Original file: `turtlebot4_gz_bringup/worlds/maze.sdf`
- Local source copied from installed `ros-jazzy-turtlebot4-gz-bringup` 2.0.2.
- The copied maze was compared with the pinned upstream file and the SHA-256 hashes matched: `c1f95e9f7874249332e2d7a2d7d7444817c35271a07e96fc235397f8697d4c30`.
- Original retained as `src/ail_tb4_maze/worlds/upstream_maze.sdf`.
- Upstream Apache-2.0 license retained as `src/ail_tb4_maze/worlds/LICENSE.upstream`. Original world / robot assets belong to their upstream authors, including Clearpath Robotics and iRobot. Installed robot assets are reused in place, not redistributed here.
- Gazebo GUI configuration is adapted from the same official simulator package, under the same license.

## Adaptation changes (privileged)

1. Rename world `maze` to `maze_escape`, avoiding ambiguity in simulator topics.
2. Replace border link `wall2` at x=10.005 with two segments, retaining its 0.01 m thickness and 2 m height. The physical opening is y=-3 to -1. No interior walls changed.
3. Add Gazebo contact sensors to all 41 boundary/interior wall links, publishing solely on the privileged transport topic `/challenge/wall_contacts`.
4. Add world Sensors (Ogre2) and Imu systems; retain original Physics, UserCommands, SceneBroadcaster and Contact systems.
5. Set physics step from 0.003 to 0.001 seconds to match the official wheel-controller update period. Normal real-time factor remains 1.
6. Change the operator GUI camera and omit upstream HMI/teleoperation widgets requiring extra helper nodes. Add entity-tree view. This is an operator view, never controller data.

Launch-time robot changes retain the official **standard TurtleBot 4** geometry, inertias, Create 3 drivetrain, encoder odometry, LiDAR and IMU. Camera/IR/cliff sensor capture definitions are removed from the instantiated URDF to reduce unnecessary rendering and prevent accidental observation; camera hardware geometry remains. Duplicate robot Contact/Sensors systems are removed because the world owns those systems. Model-pose publication is enabled for the privileged referee. No replacement robot, ground-truth odometry, SLAM, navigation stack, docking station, or maze-solving node is added.
