# AI Legion TurtleBot 4 Maze Challenge

ROS 2 package **ail_tb4_maze** for Ubuntu 24.04, ROS 2 Jazzy and Gazebo Harmonic. Includes Gazebo, RViz, live maze setup, a privileged referee and a restricted navigation interface. The supplied controller is a stationary placeholder: no maze-solving algorithm is included.

## Launch on this machine

```bash
source /opt/ros/jazzy/setup.bash
source ~/ai_legion_ws/install/setup.bash
ros2 launch ail_tb4_maze setup.launch.py
```

The setup window opens Gazebo and RViz automatically. Commands work from any directory. Close the setup window to stop the environment it started.

## Build from a repository checkout

Place this repository at `YOUR_WORKSPACE/src/ail_tb4_maze`. Install the ROS dependencies listed in package.xml with rosdep in an existing Jazzy environment, then:

```bash
source /opt/ros/jazzy/setup.bash
cd YOUR_WORKSPACE
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select ail_tb4_maze
source install/setup.bash
ros2 launch ail_tb4_maze setup.launch.py
```

`requirements.txt` lists Python dependencies; ROS and Gazebo bindings come from ROS/Ubuntu packages, not pip. On this machine PyQt5 and Gazebo bindings are already installed. The package-local `scripts/build.sh` can also use the existing local colcon fallback.

## Setup controls

- Choose the official layout or a random grid maze. Set the square maze side (12–40 m) and grid cells. Press Enter or Apply size / grid after typing, or use Randomize. The preview labels the applied physical dimensions. Changes replace walls live without reopening Gazebo and reset the trial and controller.
- The 2D preview follows the successfully applied layout. Ground uses the preferred green terrain texture; walls use brick and shrubs use green hedge textures.
- Start timer begins the 300 simulated-second trial. Collision episodes, distance, pass/fail and the saved JSON result appear in the window.
- Edit `navigation_interface/controller.py`, or enter another Python script path. Start node launches it in ROS domain 62; Kill node stops its process group. Start the timer before starting your algorithm. Commands are gated by the active trial.
- Reset robot / trial clears the attempt and stops the controller. Restart the controller for each attempt.

## Repository layout

```text
ail_tb4_maze/          Python infrastructure and GUI
launch/               setup.launch.py and challenge.launch.py
navigation_interface/ controller.py and optional AI algorithm contract
assets/               local texture assets
worlds/, config/      simulator and referee configuration
test/, scripts/       validation and development helpers
docs/                 provenance, attribution and validation notes
.runtime/             generated scenes, logs and results (ignored by Git)
.tools/               local build-tool fallback (ignored by Git)
```

Normal ROS workspace `build/`, `install/`, and `log/` directories remain at workspace level. Project scripts, documentation and runtime files live inside this package. For an installed package without a source checkout, runtime files use `$XDG_STATE_HOME/ail_tb4_maze` (default `~/.local/state/ail_tb4_maze`). Override with the `runtime_dir` setup launch argument or `AIL_TB4_MAZE_RUNTIME_DIR`.

## Want your AI to implement the algorithm?

Use [navigation_interface/CONTRACT.md](navigation_interface/CONTRACT.md) as the task brief and [controller.py](navigation_interface/controller.py) as the starting script. Give only the `navigation_interface/` directory to your AI in a fresh chat or isolated project. Ask it to implement a sensor-based maze escape algorithm inside `compute_velocity()` following the contract.

Keep world files, maze configuration, setup previews, referee results and the prior infrastructure conversation outside that AI context. The operator runs the setup GUI, starts the timer and uses Start node to run the resulting script. The placeholder itself does not solve the maze.

## Navigation and scoring

Read [the navigation contract](navigation_interface/CONTRACT.md) before implementing a controller. Only LiDAR, wheel odometry, IMU, allowed TF and simulation clock are supplied; velocity commands use TwistStamped. Infrastructure uses ROS domain 61; a gateway relays permitted data to domain 62. The referee measures simulation time, ground-truth travel, wall-contact episodes and full-footprint exit crossing. Larger layouts may require longer routes than a controller can finish in 300 seconds; geometric reachability does not guarantee completion within the time budget.

Isolation is a challenge rule and logical separation, not an OS security sandbox. For a future AI implementation, provide only the navigation_interface directory in a fresh chat, without infrastructure files, preview images or this setup conversation.

## Validation and attribution

```bash
colcon test --packages-select ail_tb4_maze
colcon test-result --verbose
```

See docs/VALIDATION.md for earlier runtime evidence; historical logs describe the package before its rename. Live maze replacement, preview synchronization, collision/exit scoring and the original 300-second timeout were exercised. Texture licenses are in docs/TEXTURE_ATTRIBUTION.md; maze provenance is in docs/MAZE_SELECTION.md and worlds/LICENSE.upstream. Project code uses Apache-2.0.
