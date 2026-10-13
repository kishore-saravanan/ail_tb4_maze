#!/usr/bin/env bash
set -eo pipefail
package_root=$(cd "$(dirname "$0")/.." && pwd)
workspace_root=${AIL_TB4_MAZE_WORKSPACE:-$(cd "$package_root/../.." && pwd)}
source /opt/ros/jazzy/setup.bash
source "$workspace_root/install/setup.bash"
export ROS_DOMAIN_ID=61 GZ_PARTITION=ai_legion_maze
mkdir -p "$package_root/.runtime/results"
printf '%s\n' "$$" > "$package_root/.runtime/results/infrastructure.pid"
exec ros2 launch ail_tb4_maze challenge.launch.py "$@"
