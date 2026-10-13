#!/usr/bin/env bash
set -eo pipefail
package_root=$(cd "$(dirname "$0")/.." && pwd)
workspace_root=${AIL_TB4_MAZE_WORKSPACE:-$(cd "$package_root/../.." && pwd)}
source /opt/ros/jazzy/setup.bash
source "$workspace_root/install/setup.bash"
cd "$workspace_root"
if command -v colcon >/dev/null; then
  colcon test --packages-select ail_tb4_maze --event-handlers console_direct+
  colcon test-result --verbose
else
  export PYTHONPATH="$package_root/.tools/python:$PYTHONPATH"
  /usr/bin/python3 -m colcon test --packages-select ail_tb4_maze --event-handlers console_direct+
  /usr/bin/python3 -m colcon test-result --verbose
fi
/usr/bin/python3 "$package_root/scripts/validate_geometry.py"
