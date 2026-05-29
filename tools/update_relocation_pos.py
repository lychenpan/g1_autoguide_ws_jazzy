#!/usr/bin/env python3
"""
Capture the current pose from ROS 2 /unitree/odom (domain 1) and save it for
g1_slam_relocation.py.

Run while the odom bridge is publishing (e.g. unitree_relocation_odom_bridge.py
after relocation is already active, or any node publishing /unitree/odom).

If /unitree/odom has no data before --timeout, writes all zeros to the JSON file.

Usage:
    python3 update_relocation_pos.py
    python3 update_relocation_pos.py --timeout 15
    python3 update_relocation_pos.py --output /path/to/relocation_init_pose.json
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

os.environ.setdefault("ROS_DOMAIN_ID", "1")

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

DEFAULT_OUTPUT = Path(__file__).resolve().parent / "relocation_init_pose.json"
ODOM_TOPIC = "/unitree/odom"

ZERO_POSE = {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0,
    "q_x": 0.0,
    "q_y": 0.0,
    "q_z": 0.0,
    "q_w": 0.0,
}


def _pose_from_odom(msg: Odometry) -> dict:
    p = msg.pose.pose.position
    o = msg.pose.pose.orientation
    return {
        "x": float(p.x),
        "y": float(p.y),
        "z": float(p.z),
        "q_x": float(o.x),
        "q_y": float(o.y),
        "q_z": float(o.z),
        "q_w": float(o.w),
        "frame_id": msg.header.frame_id,
        "child_frame_id": msg.child_frame_id,
    }


class RelocationPoseCapture(Node):
    def __init__(self, output_path: Path) -> None:
        super().__init__("relocation_pose_capture")
        self._output_path = output_path
        self._saved = False
        self._done = False
        self._sub = self.create_subscription(
            Odometry, ODOM_TOPIC, self._on_odom, 10
        )
        self.get_logger().info(
            f"Waiting for one message on {ODOM_TOPIC} (ROS_DOMAIN_ID="
            f"{os.environ.get('ROS_DOMAIN_ID', '?')}) ..."
        )

    def _write_payload(self, payload: dict, *, from_odom: bool) -> None:
        self._output_path.parent.mkdir(parents=True, exist_ok=True)
        with open(self._output_path, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2)
            f.write("\n")
        self._saved = True
        self.get_logger().info(f"Wrote {self._output_path}")
        self.get_logger().info(
            f"  pose: x={payload['x']}, y={payload['y']}, z={payload['z']}, "
            f"q=({payload['q_x']}, {payload['q_y']}, {payload['q_z']}, {payload['q_w']})"
        )

    def _write_pose(self, pose: dict) -> None:
        payload = {
            "x": pose["x"],
            "y": pose["y"],
            "z": pose["z"],
            "q_x": pose["q_x"],
            "q_y": pose["q_y"],
            "q_z": pose["q_z"],
            "q_w": pose["q_w"],
        }
        self._write_payload(payload, from_odom=True)
        if pose.get("frame_id") or pose.get("child_frame_id"):
            self.get_logger().info(
                f"  frames: {pose.get('frame_id')} -> {pose.get('child_frame_id')}"
            )

    def write_zero_pose(self) -> None:
        """Write all-zero pose when /unitree/odom has no messages."""
        self._write_payload(dict(ZERO_POSE), from_odom=False)

    def _finish(self) -> None:
        self._done = True

    def _on_odom(self, msg: Odometry) -> None:
        if self._saved:
            return
        self._write_pose(_pose_from_odom(msg))
        self._finish()

    def handle_timeout(self, timeout_sec: float) -> None:
        self.get_logger().warn(
            f"No message on {ODOM_TOPIC} within {timeout_sec}s. "
            "Is unitree_relocation_odom_bridge (or similar) running?"
        )
        self.write_zero_pose()
        self._finish()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help=f"JSON output path (default: {DEFAULT_OUTPUT})",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="Seconds to wait for /unitree/odom (default: 10)",
    )
    args = parser.parse_args()

    rclpy.init()
    node = RelocationPoseCapture(args.output)
    try:
        deadline = time.monotonic() + args.timeout
        while rclpy.ok() and not node._done and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node._saved:
            node.handle_timeout(args.timeout)
        exit_code = 0 if node._saved else 1
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
