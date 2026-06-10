#!/usr/bin/env python3
"""Publish a start message to trigger the showroom workflow mission."""

from __future__ import annotations

import argparse
import os
import sys

os.environ.setdefault("ROS_DOMAIN_ID", "1")

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

DEFAULT_TOPIC = os.environ.get("SHOWROOM_MISSION_START_TOPIC", "/showroom_mission/start")
DEFAULT_MESSAGE = "start"


class MissionStartPublisher(Node):
    def __init__(self, topic: str, message: str) -> None:
        super().__init__("mission_start_publisher")
        self._pub = self.create_publisher(String, topic, 10)
        self._message = message
        self._topic = topic

    def publish_once(self, wait_sec: float = 1.0) -> None:
        deadline = self.get_clock().now().nanoseconds + int(wait_sec * 1e9)
        while self.get_clock().now().nanoseconds < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

        msg = String()
        msg.data = self._message
        self._pub.publish(msg)
        self.get_logger().info(
            f"Published std_msgs/String to {self._topic}: {self._message!r}"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "message",
        nargs="?",
        default=DEFAULT_MESSAGE,
        help=f"Start message string (default: {DEFAULT_MESSAGE!r})",
    )
    parser.add_argument(
        "--topic",
        default=DEFAULT_TOPIC,
        help=f"Mission start topic (default: {DEFAULT_TOPIC})",
    )
    args = parser.parse_args()

    rclpy.init()
    node = MissionStartPublisher(args.topic, args.message)
    try:
        node.publish_once()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
