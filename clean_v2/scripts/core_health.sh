#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${ROOT_DIR}"

SERVICE="${SERVICE:-ros2-vnc}"

if ! command -v docker >/dev/null 2>&1; then
  echo "ERROR: docker not found"
  exit 1
fi

ARGS=("$@")
ARGS_STR=""
if [ "${#ARGS[@]}" -gt 0 ]; then
  ARGS_STR="$(printf '%q ' "${ARGS[@]}")"
fi

docker compose exec "${SERVICE}" bash -lc \
  "source /opt/ros/jazzy/setup.bash && \
   source \$ROS2_WS/install/setup.bash && \
   ros2 run mm_bringup core_health_check.sh ${ARGS_STR}"
