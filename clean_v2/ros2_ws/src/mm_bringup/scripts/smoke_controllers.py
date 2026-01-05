#!/usr/bin/env python3
# Autor: Camilo Soto Villegas
# Contacto: camilo.soto.v@usach.cl
# Proyecto: clean_v2

import sys

import rclpy
from controller_manager_msgs.srv import ListControllers
from rclpy.node import Node


class ControllersChecker(Node):
    def __init__(self):
        super().__init__('smoke_controllers')
        self.declare_parameter('namespace', 'mm1')
        self.declare_parameter('timeout_sec', 5.0)

    def run_check(self):
        namespace = self.get_parameter('namespace').value.strip('/')
        timeout = float(self.get_parameter('timeout_sec').value)
        service_name = f'/{namespace}/controller_manager/list_controllers' if namespace else '/controller_manager/list_controllers'

        client = self.create_client(ListControllers, service_name)
        if not client.wait_for_service(timeout_sec=timeout):
            print(f'FAIL: servicio no disponible: {service_name}')
            return 1

        future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None:
            print('FAIL: respuesta vacia del controller_manager')
            return 1

        controllers = {ctrl.name: ctrl.state for ctrl in future.result().controller}
        expected = {
            'joint_state_broadcaster',
            'omni_wheel_controller',
            'arm_trajectory_controller',
            'gripper_trajectory_controller',
        }

        missing = sorted(name for name in expected if name not in controllers)
        not_active = sorted(name for name in expected if controllers.get(name) != 'active')

        if missing:
            print(f'FAIL: controladores faltantes: {missing}')
            return 1
        if not_active:
            print(f'FAIL: controladores no activos: {not_active}')
            return 1

        print('PASS: controller_manager responde y controladores activos')
        return 0


def main():
    rclpy.init()
    node = ControllersChecker()
    try:
        code = node.run_check()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
