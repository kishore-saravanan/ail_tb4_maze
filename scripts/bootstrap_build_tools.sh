#!/usr/bin/env bash
# Only local Python build tools. Does not install or replace ROS or Gazebo.
set -eo pipefail
cd "$(dirname "$0")/.."
mkdir -p .tools
curl --fail --location --output .tools/pip.pyz https://bootstrap.pypa.io/pip/pip.pyz
/usr/bin/python3 .tools/pip.pyz install --target .tools/python --break-system-packages -r docs/build_tools_requirements.txt
