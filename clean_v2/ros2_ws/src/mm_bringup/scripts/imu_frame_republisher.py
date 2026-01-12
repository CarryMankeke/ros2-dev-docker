#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class ImuFrameRepublisher(Node):
    """Re-publica IMU con frame_id alineado al link de la base.

    Uso minimo:
      ros2 run mm_bringup imu_frame_republisher.py --ros-args -p prefix:=mm1_
    """

    def __init__(self):
        super().__init__('imu_frame_republisher')
        self.declare_parameter('prefix', '')
        prefix = self.get_parameter('prefix').get_parameter_value().string_value
        namespace = self.get_namespace().strip('/')
        ns_prefix = f'/{namespace}' if namespace else ''

        self._frame_id = f'{prefix}imu_link'
        src_topic = f'{ns_prefix}/imu_raw'
        dst_topic = f'{ns_prefix}/imu'

        self._pub = self.create_publisher(Imu, dst_topic, qos_profile_sensor_data)
        self.create_subscription(Imu, src_topic, self._republish, qos_profile_sensor_data)
        self.get_logger().info(f'IMU frame republisher activo: {src_topic} -> {dst_topic}')

    def _republish(self, msg: Imu):
        msg.header.frame_id = self._frame_id
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = ImuFrameRepublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
