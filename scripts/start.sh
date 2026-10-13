#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=61
exec ros2 service call /referee/start std_srvs/srv/Trigger '{}'
