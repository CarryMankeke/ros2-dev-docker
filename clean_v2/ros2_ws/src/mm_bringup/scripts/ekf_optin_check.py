#!/usr/bin/env python3
import argparse
import sys
import subprocess
import time


def _ns_prefix(namespace: str) -> str:
    return f"/{namespace}" if namespace else ""


def _run_command(args, timeout=15):
    try:
        result = subprocess.run(
            args,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout,
        )
        return result.stdout.strip()
    except subprocess.TimeoutExpired:
        return ""


def _wait_for_entry(command, match_value, retries=5, delay=1.0):
    for _ in range(retries):
        output = _run_command(command).splitlines()
        entries = set(line.strip() for line in output if line.strip())
        if match_value in entries:
            return True
        time.sleep(delay)
    return False


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--namespace", default="mm1")
    args = parser.parse_args()

    namespace = args.namespace.lstrip("/")
    ns = _ns_prefix(namespace)

    status = 0
    node_name = f"{ns}/ekf_filter_node"
    odom_topic = f"{ns}/odometry/filtered"

    if not _wait_for_entry(["ros2", "node", "list"], node_name, retries=8, delay=1.0):
        print(f"[EKF] FAIL node missing: {node_name}")
        return 1
    print(f"[EKF] PASS node: {node_name}")

    if not _wait_for_entry(["ros2", "topic", "list"], odom_topic, retries=8, delay=1.0):
        print(f"[EKF] FAIL topic missing: {odom_topic}")
        return 1

    msg = _run_command(["timeout", "5", "ros2", "topic", "echo", odom_topic, "--once"], timeout=8)
    if msg:
        print(f"[EKF] PASS {odom_topic} published")
    else:
        print(f"[EKF] WARN {odom_topic} advertised but no messages yet")

    return status


if __name__ == "__main__":
    sys.exit(main())
