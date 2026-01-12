#!/usr/bin/env python3
"""MoveIt opt-in smoke check for mm1/mm2 namespaces."""
import argparse
import sys
import time
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient


def main():
    parser = argparse.ArgumentParser(description='MoveIt opt-in check')
    parser.add_argument('--namespace', default='mm1', help='Robot namespace without leading slash')
    args = parser.parse_args()

    namespace = args.namespace.strip('/')
    move_group_name = f'{namespace}/move_group'

    rclpy.init()
    node = Node('moveit_optin_check')

    ok = True
    details = []

    time_limit = time.time() + 5.0
    nodes = []
    while time.time() < time_limit:
        nodes = node.get_node_names_and_namespaces()
        if any(name == 'move_group' and ns.strip('/') == namespace for name, ns in nodes):
            break
        time.sleep(0.2)

    if not any(name == 'move_group' and ns.strip('/') == namespace for name, ns in nodes):
        ok = False
        details.append(f'[MOVE_GROUP] FAIL node /{move_group_name} not found')
    else:
        details.append(f'[MOVE_GROUP] PASS node /{move_group_name} found')

    if ok:
        target_node = f'/{move_group_name}'
        param_client = AsyncParameterClient(node, target_node)
        if not param_client.wait_for_services(timeout_sec=5.0):
            ok = False
            details.append(f'[PARAMS] FAIL parameter service not available for {target_node}')
        else:
            try:
                future = param_client.get_parameters(['robot_description', 'robot_description_semantic'])
                rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
                result = future.result()
                values = [] if result is None else result.values
                if len(values) != 2:
                    ok = False
                    details.append('[PARAMS] FAIL missing robot_description or robot_description_semantic')
                else:
                    rd = values[0].string_value
                    rs = values[1].string_value
                    if not rd or not rs:
                        ok = False
                        details.append('[PARAMS] FAIL empty robot_description or robot_description_semantic')
                    else:
                        details.append('[PARAMS] PASS robot_description and robot_description_semantic')
            except Exception as exc:
                ok = False
                details.append(f'[PARAMS] FAIL exception {exc}')

    action_names = []
    try:
        result = subprocess.run(
            ['ros2', 'action', 'list'],
            check=False,
            capture_output=True,
            text=True,
        )
        if result.returncode == 0:
            action_names = [line.strip() for line in result.stdout.splitlines() if line.strip()]
        else:
            details.append('[ACTIONS] WARN failed to list actions via ros2 CLI')
    except Exception as exc:
        details.append(f'[ACTIONS] WARN exception listing actions: {exc}')
    arm_action = f'/{namespace}/arm_trajectory_controller/follow_joint_trajectory'
    gripper_action = f'/{namespace}/gripper_trajectory_controller/follow_joint_trajectory'

    if arm_action in action_names:
        details.append(f'[ACTIONS] PASS {arm_action}')
    else:
        details.append(f'[ACTIONS] WARN {arm_action} not found')

    if gripper_action in action_names:
        details.append(f'[ACTIONS] PASS {gripper_action}')
    else:
        details.append(f'[ACTIONS] WARN {gripper_action} not found')

    for line in details:
        print(line)

    node.destroy_node()
    rclpy.shutdown()

    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
