#!/usr/bin/env python3
"""Publish std_msgs/String to ROS 2 topics (e.g. trigger workflow.py mission start)."""

from __future__ import annotations

import os
import time

os.environ.setdefault("ROS_DOMAIN_ID", "1")

import rclpy
from std_msgs.msg import String

MISSION_START_TOPIC = os.environ.get("SHOWROOM_MISSION_START_TOPIC", "/showroom_mission/start")
DEFAULT_START_MESSAGE = "start"


class MissionStartPublisher:
    """Lazy ROS 2 publisher; safe to call from non-ROS callback threads."""

    def __init__(
        self,
        topic: str = MISSION_START_TOPIC,
        node_name: str = "mission_start_publisher",
        wait_sec: float = 1.0,
    ) -> None:
        self._topic = topic
        self._node_name = node_name
        self._wait_sec = wait_sec
        self._node = None
        self._pub = None

    def _ensure_ready(self) -> None:
        if self._node is not None:
            return
        if not rclpy.ok():
            rclpy.init()
        self._node = rclpy.create_node(self._node_name)
        self._pub = self._node.create_publisher(String, self._topic, 10)

    def publish(self, message: str = DEFAULT_START_MESSAGE) -> None:
        self._ensure_ready()
        deadline = time.monotonic() + self._wait_sec
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.1)

        msg = String()
        msg.data = message
        self._pub.publish(msg)
        self._node.get_logger().info(
            f"Published std_msgs/String to {self._topic}: {message!r}"
        )


_default_publisher = MissionStartPublisher()


def publish_mission_start(
    message: str = DEFAULT_START_MESSAGE,
    topic: str | None = None,
) -> None:
    """Publish one mission-start string (default topic: /showroom_mission/start)."""
    if topic is None or topic == MISSION_START_TOPIC:
        _default_publisher.publish(message)
        return

    MissionStartPublisher(topic=topic).publish(message)
