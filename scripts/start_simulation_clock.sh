#!/usr/bin/env bash
set -eo pipefail

PX4_GZ_WORLD="${PX4_GZ_WORLD:-trajectory_tracking}"

source /opt/ros/jazzy/setup.bash

exec ros2 run ros_gz_bridge parameter_bridge \
  "/world/${PX4_GZ_WORLD}/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock" \
  --ros-args -r __node:=simulation_clock_bridge \
  -r "/world/${PX4_GZ_WORLD}/clock:=/clock" -p use_sim_time:=false
