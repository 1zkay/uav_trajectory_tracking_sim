#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PYTHON_VENV="${PYTHON_VENV:-/home/zk/px4-venv}"

cd "${SIM_ROOT}"
if [[ -f "${PYTHON_VENV}/bin/activate" ]]; then
  source "${PYTHON_VENV}/bin/activate"
fi
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
