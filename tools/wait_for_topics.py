#!/usr/bin/env python3
"""Exit 0 once all listed ROS 2 topics publish at least one message."""

from __future__ import annotations

import argparse
import subprocess
import sys
import time


def topic_listed(topic: str) -> bool:
    try:
        result = subprocess.run(
            ["ros2", "topic", "list"],
            capture_output=True,
            text=True,
            timeout=10,
        )
    except (subprocess.TimeoutExpired, subprocess.SubprocessError, OSError):
        return False
    if result.returncode != 0:
        return False
    return topic in result.stdout.splitlines()


def topic_ready(topic: str, echo_timeout: float) -> bool:
    if not topic_listed(topic):
        return False
    try:
        result = subprocess.run(
            ["ros2", "topic", "echo", topic, "--once"],
            capture_output=True,
            timeout=echo_timeout,
        )
    except subprocess.TimeoutExpired:
        return False
    except (subprocess.SubprocessError, OSError):
        return False
    return result.returncode == 0


def wait_for_topic(topic: str, timeout_sec: float, echo_timeout: float) -> bool:
    deadline = time.monotonic() + timeout_sec
    attempt = 0
    print(f"[wait_for_topics] waiting for {topic} (timeout {timeout_sec}s)...", flush=True)
    while time.monotonic() < deadline:
        attempt += 1
        if topic_ready(topic, echo_timeout):
            print(f"[wait_for_topics] ready: {topic} (attempt {attempt})", flush=True)
            return True
        time.sleep(1.0)
    print(f"[wait_for_topics] timed out: {topic}", flush=True, file=sys.stderr)
    return False


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("topics", nargs="+", help="Topics to wait for")
    parser.add_argument("--timeout", type=float, default=60.0, help="Seconds per topic")
    parser.add_argument(
        "--echo-timeout",
        type=float,
        default=5.0,
        help="Seconds for each ros2 topic echo --once call",
    )
    args = parser.parse_args()

    for topic in args.topics:
        if not wait_for_topic(topic, args.timeout, args.echo_timeout):
            return 1
    print("[wait_for_topics] all topics ready", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
