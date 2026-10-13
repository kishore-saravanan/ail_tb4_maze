#!/usr/bin/env bash
set -eo pipefail
package_root=$(cd "$(dirname "$0")/.." && pwd)
workspace_root=${AIL_TB4_MAZE_WORKSPACE:-$(cd "$package_root/../.." && pwd)}
source /opt/ros/jazzy/setup.bash
cd "$workspace_root"
if command -v colcon >/dev/null; then
  colcon build --symlink-install --packages-select ail_tb4_maze
else
  export PYTHONPATH="$package_root/.tools/python:$PYTHONPATH"
  /usr/bin/python3 -m colcon build --symlink-install --packages-select ail_tb4_maze
fi
