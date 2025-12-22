#!/usr/bin/env python3
import pathlib

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class RobotDescriptionPublisher(Node):
    def __init__(self) -> None:
        super().__init__('robot_description_publisher')
        self.declare_parameter('urdf_file', '')
        self.declare_parameter('publish_period', 0.5)

        urdf_file = self.get_parameter('urdf_file').value
        if not urdf_file:
            self.get_logger().error('Parameter "urdf_file" is required.')
            raise RuntimeError('urdf_file parameter not set')

        self._urdf_path = pathlib.Path(urdf_file)
        self._published = False
        self._missing_warned = False

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        self._publisher = self.create_publisher(String, 'robot_description', qos)

        period = float(self.get_parameter('publish_period').value)
        self._timer = self.create_timer(max(period, 0.1), self._on_timer)

    def _on_timer(self) -> None:
        if self._published:
            self._timer.cancel()
            return

        if not self._urdf_path.exists():
            if not self._missing_warned:
                self.get_logger().warn(
                    f'URDF not found yet at {self._urdf_path}, waiting...'
                )
                self._missing_warned = True
            return

        urdf_text = self._urdf_path.read_text(encoding='utf-8')
        msg = String()
        msg.data = urdf_text
        self._publisher.publish(msg)
        self._published = True
        self.get_logger().info(f'Published robot_description from {self._urdf_path}')


def main() -> None:
    rclpy.init()
    try:
        node = RobotDescriptionPublisher()
    except RuntimeError:
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
