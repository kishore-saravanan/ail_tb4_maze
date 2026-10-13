#!/usr/bin/env bash
# Convenience wrapper; the public entry point is ros2 launch ail_tb4_maze setup.launch.py.
set -eo pipefail
package_root=$(cd "$(dirname "$0")/.." && pwd)
workspace_root=${AIL_TB4_MAZE_WORKSPACE:-$(cd "$package_root/../.." && pwd)}
source /opt/ros/jazzy/setup.bash
source "$workspace_root/install/setup.bash"
exec ros2 launch ail_tb4_maze setup.launch.py "$@"
