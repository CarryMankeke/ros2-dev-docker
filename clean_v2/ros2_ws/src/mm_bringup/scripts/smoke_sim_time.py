#!/usr/bin/env python3
# Autor: Camilo Soto Villegas
# Contacto: camilo.soto.v@usach.cl
# Proyecto: clean_v2

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


class SimTimeChecker(Node):
    def __init__(self):
        super().__init__('smoke_sim_time')
        self.declare_parameter('timeout_sec', 5.0)
        self.declare_parameter('min_samples', 3)
        self.declare_parameter('namespace', 'mm1')
        self.declare_parameter('prefix', 'mm1_')
        self._times = []

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(Clock, '/clock', self._on_clock, qos)

    def _on_clock(self, msg):
        stamp_ns = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        self._times.append(stamp_ns)

    def run_check(self):
        timeout = float(self.get_parameter('timeout_sec').value)
        min_samples = int(self.get_parameter('min_samples').value)
        start = time.time()
        while rclpy.ok() and time.time() - start < timeout and len(self._times) < min_samples:
            rclpy.spin_once(self, timeout_sec=0.1)

        if len(self._times) < min_samples:
            print('FAIL: /clock no publica suficientes mensajes')
            return 1

        monotonic = all(b > a for a, b in zip(self._times, self._times[1:]))
        if not monotonic:
            print('FAIL: /clock no es monotono')
            return 1

        print('PASS: /clock existe y es monotono')
        return 0


def main():
    rclpy.init()
    node = SimTimeChecker()
    try:
        code = node.run_check()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
