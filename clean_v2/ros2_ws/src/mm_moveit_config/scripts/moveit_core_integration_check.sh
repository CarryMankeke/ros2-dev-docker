#!/usr/bin/env bash
set -euo pipefail

namespace="mm1"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --namespace)
      namespace="$2"
      shift 2
      ;;
    *)
      echo "[USAGE] moveit_core_integration_check.sh --namespace mm1"
      exit 1
      ;;
  esac
done

ns="/${namespace}"
status=0

node_found=""
for _ in $(seq 1 15); do
  node_found=$(ros2 node list | grep -x "${ns}/move_group" || true)
  if [[ -n "$node_found" ]]; then
    break
  fi
  sleep 1
done
if [[ -n "$node_found" ]]; then
  echo "[MOVE_GROUP] PASS node ${ns}/move_group found"
else
  echo "[MOVE_GROUP] FAIL node ${ns}/move_group not found"
  status=1
fi

rd=""
rs=""
for _ in $(seq 1 15); do
  rd=$(ros2 param get "${ns}/move_group" robot_description 2>/dev/null || true)
  rs=$(ros2 param get "${ns}/move_group" robot_description_semantic 2>/dev/null || true)
  if [[ "$rd" == *"String value is:"* ]] && [[ "$rs" == *"String value is:"* ]]; then
    break
  fi
  sleep 1
done

if [[ "$rd" == *"String value is:"* ]] && [[ "$rs" == *"String value is:"* ]]; then
  echo "[PARAMS] PASS robot_description and robot_description_semantic"
else
  echo "[PARAMS] FAIL robot_description or robot_description_semantic missing"
  status=1
fi

a1=$(ros2 action list | grep -x "${ns}/arm_trajectory_controller/follow_joint_trajectory" || true)
a2=$(ros2 action list | grep -x "${ns}/gripper_trajectory_controller/follow_joint_trajectory" || true)

if [[ -n "$a1" ]]; then
  echo "[ACTIONS] PASS ${ns}/arm_trajectory_controller/follow_joint_trajectory"
else
  echo "[ACTIONS] FAIL ${ns}/arm_trajectory_controller/follow_joint_trajectory"
  status=1
fi

if [[ -n "$a2" ]]; then
  echo "[ACTIONS] PASS ${ns}/gripper_trajectory_controller/follow_joint_trajectory"
else
  echo "[ACTIONS] FAIL ${ns}/gripper_trajectory_controller/follow_joint_trajectory"
  status=1
fi

exit $status
