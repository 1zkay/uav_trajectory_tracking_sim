#!/usr/bin/env bash
set -eo pipefail

PYTHON_VENV="${PYTHON_VENV:-/home/zk/px4-venv}"

if [[ -f "${PYTHON_VENV}/bin/activate" ]]; then
  source "${PYTHON_VENV}/bin/activate"
fi

exec MicroXRCEAgent udp4 -p 8888
