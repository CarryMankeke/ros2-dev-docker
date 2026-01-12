#!/usr/bin/env python3
import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry


class OdomRelay(Node):
    def __init__(self, input_topic: str, output_topic: str) -> None:
        super().__init__('odom_relay')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._pub = self.create_publisher(Odometry, output_topic, qos)
        self.create_subscription(Odometry, input_topic, self._cb, qos)
        self.get_logger().info(f"Relaying {input_topic} -> {output_topic}")

    def _cb(self, msg: Odometry) -> None:
        self._pub.publish(msg)


def main() -> None:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--input-topic', default='omni_wheel_controller/odom')
    parser.add_argument('--output-topic', default='odom')
    args, _ = parser.parse_known_args()

    rclpy.init(args=sys.argv)
    node = OdomRelay(args.input_topic, args.output_topic)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
