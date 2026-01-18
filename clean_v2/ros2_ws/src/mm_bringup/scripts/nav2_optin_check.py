#!/usr/bin/env python3
import argparse
import sys
import time

import subprocess


def _ns_prefix(namespace: str) -> str:
    return f"/{namespace}" if namespace else ""


def _run_command(args):
    try:
        result = subprocess.run(
            args,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=30,
        )
        return result.stdout.strip()
    except subprocess.TimeoutExpired:
        return ""


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--namespace", default="mm1")
    parser.add_argument("--wait-seconds", type=float, default=20.0)
    args = parser.parse_args()

    namespace = args.namespace.lstrip("/")
    ns = _ns_prefix(namespace)

    status = 0

    required_nodes = [
        f"{ns}/controller_server",
        f"{ns}/planner_server",
        f"{ns}/bt_navigator",
        f"{ns}/behavior_server",
    ]
    optional_nodes = [
        f"{ns}/map_server",
        f"{ns}/slam_toolbox",
        f"{ns}/amcl",
    ]

    deadline = time.monotonic() + args.wait_seconds
    available_full = set()
    missing = required_nodes
    while time.monotonic() <= deadline:
        node_lines = _run_command(["ros2", "node", "list"]).splitlines()
        available_full = set(line.strip() for line in node_lines if line.strip())
        missing = [n for n in required_nodes if n not in available_full]
        if not missing:
            break
        time.sleep(1.0)

    if missing:
        print(f"[NODES] FAIL missing: {missing}")
        status = 1
    else:
        print(f"[NODES] PASS found: {required_nodes}")

    optional_missing = [n for n in optional_nodes if n not in available_full]
    if optional_missing:
        print(f"[NODES] WARN optional missing: {optional_missing}")
    else:
        print(f"[NODES] PASS optional nodes: {optional_nodes}")

    topic_lines = _run_command(["ros2", "topic", "list"]).splitlines()
    topics = set(line.strip() for line in topic_lines if line.strip())
    cmd_vel_nav2 = f"{ns}/cmd_vel_nav2"
    if cmd_vel_nav2 in topics:
        print(f"[TOPICS] PASS {cmd_vel_nav2} advertised")
    else:
        print(f"[TOPICS] WARN {cmd_vel_nav2} not advertised (no goal or nav2 inactive)")

    map_topic = f"{ns}/map"
    if map_topic in topics:
        print(f"[TOPICS] PASS {map_topic} advertised")
    else:
        print(f"[TOPICS] WARN {map_topic} not advertised (slam/map server not active)")

    return status


if __name__ == "__main__":
    sys.exit(main())
