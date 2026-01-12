#!/usr/bin/env python3
import argparse
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from controller_manager_msgs.srv import ListControllers
from rcl_interfaces.srv import GetParameters
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, Imu, JointState, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def _ns_prefix(namespace: str) -> str:
    return f"/{namespace}" if namespace else ""


def _topic_name(namespace: str, name: str) -> str:
    prefix = _ns_prefix(namespace)
    return f"{prefix}/{name}" if prefix else f"/{name}"


class CoreHealthCheck(Node):
    def __init__(self, namespace: str) -> None:
        super().__init__("core_health_check")
        self.namespace = namespace
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)

    def _wait_for_message(self, msg_type, topic: str, timeout_sec: float, qos: QoSProfile):
        msg_holder = {"msg": None}

        def _cb(msg):
            msg_holder["msg"] = msg

        sub = self.create_subscription(msg_type, topic, _cb, qos)
        start = time.time()
        while time.time() - start < timeout_sec and msg_holder["msg"] is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(sub)
        return msg_holder["msg"]

    def _check_clock(self, timeout_sec: float = 5.0):
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        stamps = []

        def _cb(msg: Clock):
            stamp = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
            if not stamps or stamp != stamps[-1]:
                stamps.append(stamp)

        sub = self.create_subscription(Clock, "/clock", _cb, qos)
        start = time.time()
        while time.time() - start < timeout_sec and len(stamps) < 2:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(sub)
        if len(stamps) < 2:
            return False, "no /clock samples or not advancing"
        if stamps[1] <= stamps[0]:
            return False, "clock not monotonic"
        return True, "clock advancing"

    def _check_use_sim_time(self, namespace: str):
        node_name = f"{_ns_prefix(namespace)}/robot_state_publisher"
        client = self.create_client(GetParameters, f"{node_name}/get_parameters")
        if not client.wait_for_service(timeout_sec=2.0):
            return None, "robot_state_publisher param service not available"
        req = GetParameters.Request(names=["use_sim_time"])
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.result() is None:
            return None, "use_sim_time query failed"
        values = future.result().values
        if not values:
            return None, "use_sim_time not returned"
        return bool(values[0].bool_value), "use_sim_time read"

    def _check_tf(self, namespace: str):
        odom = f"{namespace}_odom"
        base_footprint = f"{namespace}_base_footprint"
        base_link = f"{namespace}_base_link"
        deadline = time.time() + 2.0
        last_error = None
        while time.time() < deadline:
            try:
                if self._tf_buffer.can_transform(
                    base_footprint,
                    odom,
                    rclpy.time.Time(),
                ):
                    self._tf_buffer.lookup_transform(
                        base_footprint,
                        odom,
                        rclpy.time.Time(),
                        timeout=Duration(seconds=1.0),
                    )
                else:
                    last_error = f"{odom}->{base_footprint} not available"
                    rclpy.spin_once(self, timeout_sec=0.1)
                    continue

                if self._tf_buffer.can_transform(
                    base_link,
                    base_footprint,
                    rclpy.time.Time(),
                ):
                    self._tf_buffer.lookup_transform(
                        base_link,
                        base_footprint,
                        rclpy.time.Time(),
                        timeout=Duration(seconds=1.0),
                    )
                else:
                    last_error = f"{base_footprint}->{base_link} not available"
                    rclpy.spin_once(self, timeout_sec=0.1)
                    continue

                return True, f"{odom}->{base_footprint}, {base_footprint}->{base_link}"
            except TransformException as exc:
                last_error = str(exc)
                rclpy.spin_once(self, timeout_sec=0.1)
        return False, f"TF lookup failed: {last_error}"

    def _list_controllers(self, namespace: str):
        service = f"{_ns_prefix(namespace)}/controller_manager/list_controllers"
        client = self.create_client(ListControllers, service)
        if not client.wait_for_service(timeout_sec=2.0):
            return None, f"service not available: {service}"
        req = ListControllers.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.result() is None:
            return None, "list_controllers failed"
        controllers = {c.name: c.state for c in future.result().controller}
        return controllers, "ok"

    def _check_joint_states(self, namespace: str):
        topic = _topic_name(namespace, "joint_states")
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        msg = self._wait_for_message(JointState, topic, 2.0, qos)
        if msg is None:
            return False, f"no messages on {topic}"
        return True, f"{topic} ok ({len(msg.name)} joints)"

    def _check_sensor_topic(self, topic: str, msg_type, expected_prefix: str):
        topics = {name for name, _ in self.get_topic_names_and_types()}
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        if topic not in topics:
            return False, f"not advertised: {topic}"
        msg = self._wait_for_message(msg_type, topic, 2.0, qos)
        if msg is None:
            return False, f"no messages within timeout: {topic}"
        if hasattr(msg, "header"):
            frame_id = msg.header.frame_id
            if not frame_id:
                return False, f"empty frame_id: {topic}"
            if expected_prefix and not frame_id.startswith(expected_prefix):
                return False, f"frame_id mismatch: {topic} ({frame_id})"
        return True, f"{topic}"

    def _check_sensors(self, namespace: str):
        expected_prefix = f"{namespace}_"
        checks = [
            (LaserScan, _topic_name(namespace, "scan")),
            (Imu, _topic_name(namespace, "imu")),
            (Imu, _topic_name(namespace, "imu/ee")),
            (Image, _topic_name(namespace, "camera/front/image_raw")),
            (Image, _topic_name(namespace, "camera/left/image_raw")),
            (Image, _topic_name(namespace, "camera/right/image_raw")),
            (Image, _topic_name(namespace, "camera/rear/image_raw")),
            (Image, _topic_name(namespace, "camera/ee/image_raw")),
        ]
        found = []
        warnings = []
        for msg_type, topic in checks:
            ok, detail = self._check_sensor_topic(topic, msg_type, expected_prefix)
            if ok:
                found.append(detail)
            else:
                warnings.append(detail)
        if warnings:
            return "WARN", f"found: {found}; issues: {warnings}"
        return "PASS", f"found: {found}"


def _print(section: str, status: str, detail: str) -> None:
    print(f"[{section}] {status} {detail}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--namespace", default="mm1")
    parser.add_argument("--check-mm2", action="store_true")
    args = parser.parse_args()

    namespace = args.namespace.lstrip("/")
    check_mm2 = args.check_mm2

    rclpy.init()
    node = CoreHealthCheck(namespace)

    exit_code = 0

    ok_clock, clock_detail = node._check_clock()
    _print("SIM_TIME", "PASS" if ok_clock else "FAIL", clock_detail)
    if not ok_clock:
        exit_code = 1

    use_sim, use_sim_detail = node._check_use_sim_time(namespace)
    if use_sim is True:
        _print("SIM_TIME", "PASS", f"use_sim_time true on /{namespace}/robot_state_publisher")
    elif use_sim is False:
        _print("SIM_TIME", "FAIL", "use_sim_time false on robot_state_publisher")
        exit_code = 1
    else:
        _print("SIM_TIME", "WARN", use_sim_detail)

    tf_ok, tf_detail = node._check_tf(namespace)
    tf_status = "PASS" if tf_ok else "FAIL"
    tf_msg = f"{namespace}: {tf_detail}"

    if check_mm2:
        tf_ok_mm2, tf_detail_mm2 = node._check_tf("mm2")
        tf_msg = f"{tf_msg}; mm2: {tf_detail_mm2}"
        if not tf_ok_mm2:
            tf_ok = False
    _print("TF", "PASS" if tf_ok else "FAIL", tf_msg)
    if not tf_ok:
        exit_code = 1

    controllers, controllers_detail = node._list_controllers(namespace)
    if controllers is None:
        _print("CONTROLLERS", "FAIL", controllers_detail)
        exit_code = 1
    else:
        required = ["joint_state_broadcaster", "omni_wheel_controller"]
        optional = ["arm_trajectory_controller", "gripper_trajectory_controller"]
        missing = [c for c in required if c not in controllers]
        inactive = [c for c in required if controllers.get(c) != "active"]
        warn = []
        for c in optional:
            if c in controllers and controllers[c] != "active":
                inactive.append(c)
            if c not in controllers:
                warn.append(f"{c} not present")
        if missing or inactive:
            _print(
                "CONTROLLERS",
                "FAIL",
                f"missing: {missing}; inactive: {inactive}",
            )
            exit_code = 1
        else:
            detail = f"active: {required}"
            if warn:
                detail = f"{detail}; warn: {warn}"
            _print("CONTROLLERS", "PASS", detail)

    js_ok, js_detail = node._check_joint_states(namespace)
    if not js_ok:
        _print("CONTROLLERS", "FAIL", js_detail)
        exit_code = 1

    if check_mm2:
        controllers_mm2, detail_mm2 = node._list_controllers("mm2")
        if controllers_mm2 is None:
            _print("CONTROLLERS", "FAIL", f"mm2: {detail_mm2}")
            exit_code = 1
        else:
            required = ["joint_state_broadcaster", "omni_wheel_controller"]
            missing = [c for c in required if c not in controllers_mm2]
            inactive = [c for c in required if controllers_mm2.get(c) != "active"]
            if missing or inactive:
                _print(
                    "CONTROLLERS",
                    "FAIL",
                    f"mm2 missing: {missing}; inactive: {inactive}",
                )
                exit_code = 1
            else:
                _print("CONTROLLERS", "PASS", "mm2 controllers ok")
        js_ok_mm2, js_detail_mm2 = node._check_joint_states("mm2")
        if not js_ok_mm2:
            _print("CONTROLLERS", "FAIL", f"mm2 {js_detail_mm2}")
            exit_code = 1

    sensors_status, sensors_detail = node._check_sensors(namespace)
    _print("SENSORS", sensors_status, sensors_detail)

    node.destroy_node()
    rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
