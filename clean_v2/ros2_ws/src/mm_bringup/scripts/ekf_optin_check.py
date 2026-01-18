#!/usr/bin/env python3
import argparse
import sys
import subprocess


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


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--namespace", default="mm1")
    args = parser.parse_args()

    namespace = args.namespace.lstrip("/")
    ns = _ns_prefix(namespace)

    status = 0
    node_name = f"{ns}/ekf_filter_node"
    odom_topic = f"{ns}/odometry/filtered"

    node_lines = _run_command(["ros2", "node", "list"]).splitlines()
    nodes = set(line.strip() for line in node_lines if line.strip())
    if node_name not in nodes:
        print(f"[EKF] FAIL node missing: {node_name}")
        return 1
    print(f"[EKF] PASS node: {node_name}")

    topic_lines = _run_command(["ros2", "topic", "list"]).splitlines()
    topics = set(line.strip() for line in topic_lines if line.strip())
    if odom_topic not in topics:
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
