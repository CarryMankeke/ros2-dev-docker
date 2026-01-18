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
   colcon build --symlink-install --packages-select mm_bringup mm_moveit_config mm_robot_description mm_arm_description"

# Clean previous processes

docker compose exec "${SERVICE}" bash -lc \
  "pkill -f '[r]os2 launch mm_bringup sim_mm.launch.py' || true; \
   pkill -f '[r]os2 launch mm_moveit_config moveit.launch.py' || true; \
   pkill -f '[g]z sim' || true"

# Launch core

docker compose exec "${SERVICE}" bash -lc \
  "source /opt/ros/jazzy/setup.bash && \
   source \$ROS2_WS/install/setup.bash && \
   nohup ros2 launch mm_bringup sim_mm.launch.py headless:=${HEADLESS} > /tmp/sim_mm.log 2>&1 &"

sleep 15

# Launch MoveIt opt-in

docker compose exec "${SERVICE}" bash -lc \
  "source /opt/ros/jazzy/setup.bash && \
   source \$ROS2_WS/install/setup.bash && \
   nohup ros2 launch mm_moveit_config moveit.launch.py namespace:=${NAMESPACE} prefix:=${PREFIX} use_sim_time:=true use_rviz:=false > /tmp/moveit.log 2>&1 &"

sleep 10

# MoveIt + core integration check

docker compose exec "${SERVICE}" bash -lc \
  "source /opt/ros/jazzy/setup.bash && \
   source \$ROS2_WS/install/setup.bash && \
   ros2 run mm_moveit_config moveit_core_integration_check.sh --namespace ${NAMESPACE}"

# Core gate

docker compose exec "${SERVICE}" bash -lc \
  "source /opt/ros/jazzy/setup.bash && \
   source \$ROS2_WS/install/setup.bash && \
   ros2 run mm_bringup core_health_check.sh --namespace ${NAMESPACE}"

# Cleanup

docker compose exec "${SERVICE}" bash -lc \
  "pkill -f '[r]os2 launch mm_moveit_config moveit.launch.py' || true; \
   pkill -f '[r]os2 launch mm_bringup sim_mm.launch.py' || true; \
   pkill -f '[g]z sim' || true"
