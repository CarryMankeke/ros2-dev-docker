#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${ROOT_DIR}"

SERVICE="${SERVICE:-ros2-vnc}"
NAMESPACE="${NAMESPACE:-mm1}"
PREFIX="${PREFIX:-mm1_}"
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
   colcon build --symlink-install --packages-select mm_bringup"

# Clean previous processes

docker compose exec "${SERVICE}" bash -lc \
  "pkill -f 'ros2 launch mm_bringup sim_mm.launch.py' || true; \
   pkill -f 'ros2 launch mm_bringup ekf.launch.py' || true; \
   pkill -f 'gz sim' || true"

# Launch sim

docker compose exec "${SERVICE}" bash -lc \
  "source /opt/ros/jazzy/setup.bash && \
   source \$ROS2_WS/install/setup.bash && \
   nohup ros2 launch mm_bringup sim_mm.launch.py headless:=${HEADLESS} > /tmp/sim_mm.log 2>&1 &"

sleep 10

# Launch EKF opt-in

docker compose exec "${SERVICE}" bash -lc \
  "source /opt/ros/jazzy/setup.bash && \
   source \$ROS2_WS/install/setup.bash && \
   nohup ros2 launch mm_bringup ekf.launch.py namespace:=${NAMESPACE} prefix:=${PREFIX} use_sim_time:=true > /tmp/ekf.log 2>&1 &"

sleep 6

# EKF check

docker compose exec "${SERVICE}" bash -lc \
  "source /opt/ros/jazzy/setup.bash && \
   source \$ROS2_WS/install/setup.bash && \
   ros2 run mm_bringup ekf_optin_check.py --namespace ${NAMESPACE}"

# Core gate

docker compose exec "${SERVICE}" bash -lc \
  "source /opt/ros/jazzy/setup.bash && \
   source \$ROS2_WS/install/setup.bash && \
   ros2 run mm_bringup core_health_check.sh --namespace ${NAMESPACE}"

# Cleanup

docker compose exec "${SERVICE}" bash -lc \
  "pkill -f 'ros2 launch mm_bringup ekf.launch.py' || true; \
   pkill -f 'ros2 launch mm_bringup sim_mm.launch.py' || true; \
   pkill -f 'gz sim' || true"
