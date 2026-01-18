#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${ROOT_DIR}"

SERVICE="${SERVICE:-ros2-vnc}"
HEADLESS="${HEADLESS:-true}"

if ! command -v docker >/dev/null 2>&1; then
  echo "ERROR: docker not found"
  exit 1
fi

if ! docker compose ps --status running | grep -q "${SERVICE}"; then
  docker compose up -d
fi

docker compose exec "${SERVICE}" bash -lc \
  "source /opt/ros/jazzy/setup.bash && \
   cd \$ROS2_WS && \
   colcon build --symlink-install --packages-select mm_bringup mm_robot_description"

# Clean previous processes

docker compose exec "${SERVICE}" bash -lc \
  "pkill -f '[r]os2 launch mm_bringup sim_mm_dual.launch.py' || true; \
   pkill -f '[g]z sim' || true"

# Launch dual sim

docker compose exec "${SERVICE}" bash -lc \
  "source /opt/ros/jazzy/setup.bash && \
   source \$ROS2_WS/install/setup.bash && \
   nohup ros2 launch mm_bringup sim_mm_dual.launch.py headless:=${HEADLESS} > /tmp/sim_mm_dual.log 2>&1 &"

sleep 15

# Core health check for mm1 + mm2

docker compose exec "${SERVICE}" bash -lc \
  "source /opt/ros/jazzy/setup.bash && \
   source \$ROS2_WS/install/setup.bash && \
   ros2 run mm_bringup core_health_check.sh --namespace mm1 --check-mm2"

# Cleanup

docker compose exec "${SERVICE}" bash -lc \
  "pkill -f '[r]os2 launch mm_bringup sim_mm_dual.launch.py' || true; \
   pkill -f '[g]z sim' || true"
