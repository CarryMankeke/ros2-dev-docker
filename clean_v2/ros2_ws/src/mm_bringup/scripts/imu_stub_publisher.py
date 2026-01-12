#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class ImuStubPublisher(Node):
    """Publica IMU basica si no llegan mensajes desde imu_raw."""

    def __init__(self):
        super().__init__('imu_stub_publisher')
        self.declare_parameter('prefix', '')
        prefix = self.get_parameter('prefix').get_parameter_value().string_value
        namespace = self.get_namespace().strip('/')
        ns_prefix = f'/{namespace}' if namespace else ''

        self._frame_id = f'{prefix}imu_link'
        self._src_topic = f'{ns_prefix}/imu_raw'
        self._dst_topic = f'{ns_prefix}/imu'

        self._last_raw = 0.0
        self._pub = self.create_publisher(Imu, self._dst_topic, qos_profile_sensor_data)
        self.create_subscription(Imu, self._src_topic, self._raw_cb, qos_profile_sensor_data)
        self.create_timer(0.05, self._tick)

    def _raw_cb(self, _msg: Imu):
        self._last_raw = time.time()

    def _tick(self):
        if time.time() - self._last_raw < 1.0:
            return
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = ImuStubPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
