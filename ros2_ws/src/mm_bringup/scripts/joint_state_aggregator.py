#!/usr/bin/env python3
"""
Agregador simple de JointState para combinar base y brazo en un tópico global.
Pensado para simulación: fusiona /mm_base/joint_states y /mm_arm/joint_states
en /joint_states, manteniendo stamps coherentes con use_sim_time.
"""

from typing import Dict, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


class JointStateAggregator(Node):
    def __init__(self) -> None:
        super().__init__('joint_state_aggregator')

        self.declare_parameter('base_joint_topic', '/mm_base/joint_states')
        self.declare_parameter('arm_joint_topic', '/mm_arm/joint_states')
        self.declare_parameter('output_joint_topic', '/joint_states')
        self.declare_parameter('publish_rate_hz', 50.0)

        base_topic = self.get_parameter('base_joint_topic').get_parameter_value().string_value
        arm_topic = self.get_parameter('arm_joint_topic').get_parameter_value().string_value
        self._output_topic = self.get_parameter('output_joint_topic').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

        qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        self._last_msgs: Dict[str, JointState] = {}
        self._sub_base = self.create_subscription(JointState, base_topic, self._on_joint_state, qos)
        self._sub_arm = self.create_subscription(JointState, arm_topic, self._on_joint_state, qos)
        self._pub = self.create_publisher(JointState, self._output_topic, qos)

        timer_period = 1.0 / publish_rate if publish_rate > 0.0 else 0.02
        self._timer = self.create_timer(timer_period, self._publish_fused)

    def _on_joint_state(self, msg: JointState) -> None:
        # Identificar por nombre del primer joint para evitar mezclar mensajes.
        key = msg.name[0] if msg.name else str(id(msg))
        self._last_msgs[key] = msg

    def _publish_fused(self) -> None:
        if not self._last_msgs:
            return

        names: List[str] = []
        positions: List[float] = []
        velocities: List[float] = []
        efforts: List[float] = []

        for msg in self._last_msgs.values():
            names.extend(msg.name)
            positions.extend(msg.position)
            velocities.extend(msg.velocity)
            efforts.extend(msg.effort)

        fused = JointState()
        fused.header.stamp = self.get_clock().now().to_msg()
        fused.name = names
        fused.position = positions
        fused.velocity = velocities
        fused.effort = efforts

        self._pub.publish(fused)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointStateAggregator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
