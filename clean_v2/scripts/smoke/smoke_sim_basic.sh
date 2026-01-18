#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${ROOT_DIR}"

SERVICE="${SERVICE:-ros2-vnc}"
NAMESPACE="${NAMESPACE:-mm1}"
HEADLESS="${HEADLESS:-true}"

if ! command -v docker >/dev/null 2>&1; then
  echo "ERROR: docker not found"
  exit 1
fi

# Ensure container is up
if ! docker compose ps --status running | grep -q "${SERVICE}"; then
  docker compose up -d
fi

# Build core packages

docker compose exec "${SERVICE}" bash -lc \
  "source /opt/ros/jazzy/setup.bash && \
   cd \$ROS2_WS && \
   colcon build --symlink-install --packages-select mm_bringup mm_robot_description mm_base_description mm_arm_description"

# Clean previous processes

docker compose exec "${SERVICE}" bash -lc \
  "pkill -f 'ros2 launch mm_bringup sim_mm.launch.py' || true; \
   pkill -f 'gz sim' || true"

# Launch sim (headless by default)

docker compose exec "${SERVICE}" bash -lc \
  "source /opt/ros/jazzy/setup.bash && \
   source \$ROS2_WS/install/setup.bash && \
   nohup ros2 launch mm_bringup sim_mm.launch.py headless:=${HEADLESS} > /tmp/sim_mm.log 2>&1 &"

sleep 10

# Run core health gate

docker compose exec "${SERVICE}" bash -lc \
  "source /opt/ros/jazzy/setup.bash && \
   source \$ROS2_WS/install/setup.bash && \
   ros2 run mm_bringup core_health_check.sh --namespace ${NAMESPACE}"

# Cleanup

docker compose exec "${SERVICE}" bash -lc \
  "pkill -f 'ros2 launch mm_bringup sim_mm.launch.py' || true; \
   pkill -f 'gz sim' || true"
