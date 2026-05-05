#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${SIM_ROOT}"
source /opt/ros/jazzy/setup.bash
source "${SIM_ROOT}/install/setup.bash"

RVIZ_CONFIG="${RVIZ_CONFIG:-${SIM_ROOT}/src/uav_trajectory_tracking/rviz/trajectory_tracking.rviz}"

exec rviz2 -d "${RVIZ_CONFIG}"
