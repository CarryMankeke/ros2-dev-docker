#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class LidarFrameRepublisher(Node):
    """Re-publica LaserScan con frame_id alineado al link del lidar.

    Uso minimo:
      ros2 run mm_bringup lidar_frame_republisher.py --ros-args -p prefix:=mm1_
    """

    def __init__(self):
        super().__init__('lidar_frame_republisher')
        self.declare_parameter('prefix', '')
        self.declare_parameter('frame_id', '')
        self.declare_parameter('src_topic', '')
        self.declare_parameter('dst_topic', '')
        prefix = self.get_parameter('prefix').get_parameter_value().string_value
        namespace = self.get_namespace().strip('/')
        ns_prefix = f'/{namespace}' if namespace else ''

        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        src_topic = self.get_parameter('src_topic').get_parameter_value().string_value
        dst_topic = self.get_parameter('dst_topic').get_parameter_value().string_value

        if not frame_id:
            frame_id = f'{prefix}lidar_link'
        elif prefix and not frame_id.startswith(prefix):
            frame_id = f'{prefix}{frame_id}'
        if not src_topic:
            src_topic = f'{ns_prefix}/scan_raw'
        if not dst_topic:
            dst_topic = f'{ns_prefix}/scan'

        self._frame_id = frame_id

        self._pub = self.create_publisher(LaserScan, dst_topic, qos_profile_sensor_data)
        self.create_subscription(LaserScan, src_topic, self._republish, qos_profile_sensor_data)
        self.get_logger().info(f'LiDAR frame republisher activo: {src_topic} -> {dst_topic} ({self._frame_id})')

    def _republish(self, msg: LaserScan):
        msg.header.frame_id = self._frame_id
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = LidarFrameRepublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
