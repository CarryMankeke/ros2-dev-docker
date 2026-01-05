#!/usr/bin/env python3
# Autor: Camilo Soto Villegas
# Contacto: camilo.soto.v@usach.cl
# Proyecto: clean_v2

import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


class TfChecker(Node):
    def __init__(self):
        super().__init__('smoke_tf')
        self.declare_parameter('prefix', 'mm1_')
        self.declare_parameter('timeout_sec', 2.0)
        self.declare_parameter('sample_sec', 2.0)
        self._parents = {}

        self._buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._listener = TransformListener(self._buffer, self)

        tf_qos = QoSProfile(depth=100)
        tf_qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(TFMessage, '/tf', self._on_tf, tf_qos)

        tf_static_qos = QoSProfile(depth=100)
        tf_static_qos.reliability = ReliabilityPolicy.RELIABLE
        tf_static_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(TFMessage, '/tf_static', self._on_tf, tf_static_qos)

    def _on_tf(self, msg):
        for transform in msg.transforms:
            child = transform.child_frame_id
            parent = transform.header.frame_id
            if child not in self._parents:
                self._parents[child] = set()
            self._parents[child].add(parent)

    def _check_multiple_parents(self):
        multi = {child: parents for child, parents in self._parents.items() if len(parents) > 1}
        if multi:
            print('FAIL: TF con multiples padres')
            for child, parents in sorted(multi.items()):
                print(f'- {child}: {sorted(parents)}')
            return False
        return True

    def _check_transform(self, target, source, timeout):
        return self._buffer.can_transform(target, source, rclpy.time.Time(), timeout=Duration(seconds=timeout))

    def run_check(self):
        prefix = self.get_parameter('prefix').value
        timeout = float(self.get_parameter('timeout_sec').value)
        sample_sec = float(self.get_parameter('sample_sec').value)

        start = time.time()
        while rclpy.ok() and time.time() - start < sample_sec:
            rclpy.spin_once(self, timeout_sec=0.1)

        if not self._check_multiple_parents():
            return 1

        checks = [
            (f'{prefix}odom', f'{prefix}base_footprint'),
            (f'{prefix}base_footprint', f'{prefix}base_link'),
            (f'{prefix}base_link', f'{prefix}cam_front_link'),
            (f'{prefix}tool0', f'{prefix}ee_cam_link'),
        ]

        missing = []
        for target, source in checks:
            if not self._check_transform(target, source, timeout):
                missing.append((target, source))

        if missing:
            print('FAIL: transforms faltantes')
            for target, source in missing:
                print(f'- {target} -> {source}')
            return 1

        print('PASS: TF critico disponible y sin dobles padres')
        return 0


def main():
    rclpy.init()
    node = TfChecker()
    try:
        code = node.run_check()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
