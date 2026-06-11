#!/usr/bin/env python3
"""
Compare /unitree/odom header.stamp vs local ROS clock and print age directly.

Usage:
    python3 check_odom_stamp_age.py              # continuous until Ctrl+C
    python3 check_odom_stamp_age.py --count 5    # print 5 messages then exit
    python3 check_odom_stamp_age.py --timeout 10 # startup wait for first message
"""

from __future__ import annotations

import argparse
import os
import sys
import time

os.environ.setdefault("ROS_DOMAIN_ID", "1")

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time

ODOM_TOPIC = "/unitree/odom"


def _format_time(stamp: Time) -> str:
    sec = stamp.nanoseconds // 1_000_000_000
    nsec = stamp.nanoseconds % 1_000_000_000
    return f"{sec}.{nsec:09d}"


class OdomStampAgeChecker(Node):
    def __init__(self, count: int | None) -> None:
        super().__init__("odom_stamp_age_checker")
        self._remaining = count
        self._received = 0
        self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, 10)

    @property
    def done(self) -> bool:
        return self._remaining is not None and self._received >= self._remaining

    def _on_odom(self, msg: Odometry) -> None:
        if self.done:
            return

        pose_stamp = Time.from_msg(msg.header.stamp)
        now = self.get_clock().now()
        age_sec = (now - pose_stamp).nanoseconds / 1e9

        print(
            f"pose_stamp={_format_time(pose_stamp)}  "
            f"ros_now={_format_time(now)}  "
            f"age={age_sec:.3f}s",
            flush=True,
        )

        self._received += 1


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Print age between /unitree/odom header.stamp and ROS clock."
    )
    parser.add_argument(
        "--count",
        type=int,
        default=None,
        help="Stop after N messages (default: run until Ctrl+C)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="Seconds to wait for the first /unitree/odom message (default: 10)",
    )
    args = parser.parse_args()

    if args.count is not None and args.count < 1:
        print("error: --count must be >= 1", file=sys.stderr)
        return 2

    rclpy.init()
    node = OdomStampAgeChecker(args.count)
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and not node.done and node._received == 0:
            if time.monotonic() >= deadline:
                break
            rclpy.spin_once(node, timeout_sec=0.1)

        if node._received == 0:
            print(
                f"error: no message on {ODOM_TOPIC} within {args.timeout:.1f}s",
                file=sys.stderr,
            )
            return 1

        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
