#!/usr/bin/env python3
"""Navigation test: yaw spin loop at each of three hardcoded waypoints.

At each waypoint:
  1. Loop 5 times: read current (ax, ay, yaw), navigate to (ax, ay, yaw + 100°)
     using two-stage navigation (rotate / translate / rotate).
  2. Navigate to the next waypoint (two-stage) and repeat.

Set SHOWROOM_AUTO_START=1 to run immediately on launch.
"""

from __future__ import annotations

import math
import os
import sys
import threading
import time
from typing import NamedTuple

_TOOLS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools")
if _TOOLS_DIR not in sys.path:
    sys.path.insert(0, os.path.normpath(_TOOLS_DIR))

from unitree_sdk2py.core.channel import ChannelFactoryInitialize  # noqa: E402

ChannelFactoryInitialize(0, os.environ.get("UNITREE_NET_IFACE", "eth0"))
os.environ.setdefault("ROS_DOMAIN_ID", "1")

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String

MISSION_START_TOPIC = os.environ.get(
    "SHOWROOM_MISSION_START_TOPIC", "/showroom_mission/start"
)

NAV_TWO_STAGE = True
if "NAV_TWO_STAGE" in os.environ:
    NAV_TWO_STAGE = os.environ["NAV_TWO_STAGE"].lower() in ("1", "true", "yes")

ODOM_TOPIC = "/unitree/odom"
DEFAULT_NAV_ALIGN_THRESH_DEG = 50.0
MIN_PATH_DIST_M = 0.05
FINAL_YAW_SKIP_THRESH_DEG = 10

# Hardcoded test waypoints: (x, y, yaw) in map frame.
TEST_POINTS: list[tuple[float, float, float]] = [
    (-7.2, 15.1, -3.111007),
    (-6.7, 18.5, -0.566922)
]

YAW_INCREMENT_DEG = float(os.environ.get("WORKFLOW_TEST_YAW_INCREMENT_DEG", "100"))
LOOP_COUNT = int(os.environ.get("WORKFLOW_TEST_LOOP_COUNT", "5"))
NAV_TIMEOUT_SEC = float(os.environ.get("WORKFLOW_TEST_NAV_TIMEOUT_SEC", "300"))


def nav_status_to_text(status: int) -> str:
    status_map = {
        GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
        GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
        GoalStatus.STATUS_EXECUTING: "EXECUTING",
        GoalStatus.STATUS_CANCELING: "CANCELING",
        GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
        GoalStatus.STATUS_CANCELED: "CANCELED",
        GoalStatus.STATUS_ABORTED: "ABORTED",
    }
    return status_map.get(status, f"UNMAPPED({status})")


def yaw_to_quat(yaw: float):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def angle_diff_deg(a: float, b: float) -> float:
    return abs(math.degrees(normalize_angle(a - b)))


def path_yaw_from_points(ax: float, ay: float, bx: float, by: float) -> float:
    return math.atan2(by - ay, bx - ax)


def yaw_deg360(yaw_rad: float) -> float:
    """Convert yaw radians to [0, 360) degrees for logging."""
    return (math.degrees(yaw_rad) + 360.0) % 360.0


class RobotPose(NamedTuple):
    x: float
    y: float
    yaw: float
    stamp: Time


class WorkflowTestNode(Node):
    """Run yaw-in-place loops at three waypoints using two-stage nav."""

    def __init__(self):
        super().__init__("workflow_test")
        self._running = False
        self._pending_start = False
        self._start_message = ""
        self._mission_thread: threading.Thread | None = None
        self._nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._latest_pose: RobotPose | None = None
        self._odom_sub = self.create_subscription(
            Odometry, ODOM_TOPIC, self._on_odom, 10
        )
        self.create_subscription(String, MISSION_START_TOPIC, self._on_start_request, 10)
        self.create_timer(0.2, self._mission_timer_cb)
        self.get_logger().info(
            f"Workflow test loaded: {len(TEST_POINTS)} waypoints, "
            f"{LOOP_COUNT} yaw loops per stop, "
            f"yaw increment={YAW_INCREMENT_DEG}°"
        )
        self.get_logger().info(
            f"NAV_TWO_STAGE={'on' if NAV_TWO_STAGE else 'off'} "
            f"({'3-step path-yaw nav' if NAV_TWO_STAGE else 'direct goal'})"
        )
        self.get_logger().info(
            f"Publish std_msgs/String to {MISSION_START_TOPIC} to begin test"
        )

    def _on_odom(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self._latest_pose = RobotPose(
            float(pos.x),
            float(pos.y),
            quat_to_yaw(ori.x, ori.y, ori.z, ori.w),
            Time.from_msg(msg.header.stamp),
        )

    def _format_ros_time(self, stamp: Time) -> str:
        sec = stamp.nanoseconds // 1_000_000_000
        nsec = stamp.nanoseconds % 1_000_000_000
        return f"{sec}.{nsec:09d}"

    def _pose_time_log_suffix(self, pose_stamp: Time) -> str:
        now = self.get_clock().now()
        age_sec = (now - pose_stamp).nanoseconds / 1e9
        return (
            f"pose_stamp={self._format_ros_time(pose_stamp)}, "
            f"now={self._format_ros_time(now)}, "
            f"age={age_sec:.3f}s"
        )

    def _get_robot_pose(self, timeout_sec: float = 5.0) -> RobotPose | None:
        if self._latest_pose is not None:
            return self._latest_pose
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._latest_pose is not None:
                return self._latest_pose
        return None

    def _on_start_request(self, msg: String) -> None:
        if self._running:
            self.get_logger().warn("Test already running, ignoring start request")
            return
        self._start_message = msg.data.strip()
        self.get_logger().info(f"Start request received: {self._start_message!r}")
        self._pending_start = True

    def _mission_timer_cb(self) -> None:
        if not self._pending_start or self._running:
            return
        self._pending_start = False
        self._running = True
        self._mission_thread = threading.Thread(
            target=self._run_mission_wrapper,
            name="workflow-test-thread",
            daemon=True,
        )
        self._mission_thread.start()

    def _run_mission_wrapper(self) -> None:
        try:
            self._run_mission()
        except Exception as exc:
            self.get_logger().exception(f"Test crashed: {exc}")
        finally:
            self._running = False

    def _send_nav_goal_blocking(
        self,
        x: float,
        y: float,
        yaw: float,
        timeout_sec: float,
        label: str = "",
    ):
        self.get_logger().info("Step[NAV]: waiting for /navigate_to_pose action server")
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("navigate_to_pose server not available")
            return None

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        qz, qw = yaw_to_quat(yaw)
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        prefix = f"{label} " if label else ""
        self.get_logger().info(
            f"Step[NAV]: {prefix}sending goal x={x:.5f}, y={y:.5f}, "
            f"yaw={yaw_deg360(yaw):.1f}°"
        )
        send_future = self._nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout_sec)
        if not send_future.done():
            self.get_logger().error("timeout waiting goal acceptance")
            return None
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("goal rejected")
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        if not result_future.done():
            self.get_logger().error("timeout waiting goal result")
            return None

        wrapped = result_future.result()
        self.get_logger().info(
            f"Step[NAV]: {prefix}status={wrapped.status}"
            f"({nav_status_to_text(wrapped.status)})"
        )
        return wrapped.status, wrapped.result

    def navigate_blocking(
        self,
        x: float,
        y: float,
        yaw: float,
        timeout_sec: float = 300.0,
        align_threshold_deg: float | None = None,
    ):
        """Navigate to (x, y, yaw). Single goal if NAV_TWO_STAGE is False, else 3-step."""
        if not NAV_TWO_STAGE:
            return self._send_nav_goal_blocking(x, y, yaw, timeout_sec)

        if align_threshold_deg is None:
            align_threshold_deg = float(
                os.environ.get(
                    "SHOWROOM_NAV_ALIGN_THRESH_DEG",
                    str(DEFAULT_NAV_ALIGN_THRESH_DEG),
                )
            )

        target_yaw = yaw
        pose = self._get_robot_pose()
        if pose is None:
            self.get_logger().error(f"No pose on {ODOM_TOPIC}, cannot navigate")
            return None

        ax, ay, robot_yaw = pose.x, pose.y, pose.yaw
        dist = math.hypot(x - ax, y - ay)
        if dist >= MIN_PATH_DIST_M:
            line_yaw = path_yaw_from_points(ax, ay, x, y)
        else:
            line_yaw = robot_yaw
            self.get_logger().info(
                f"Step[NAV]: already near target ({dist:.3f} m), skip path alignment"
            )

        self.get_logger().info(
            f"Step[NAV]: A=({ax:.5f}, {ay:.5f}, yaw={yaw_deg360(robot_yaw):.1f}°), "
            f"B=({x:.5f}, {y:.5f}, yaw={yaw_deg360(target_yaw):.1f}°), "
            f"path_yaw={yaw_deg360(line_yaw):.1f}°, dist={dist:.3f} m, "
            f"{self._pose_time_log_suffix(pose.stamp)}"
        )

        yaw_delta = angle_diff_deg(robot_yaw, line_yaw)
        if dist >= MIN_PATH_DIST_M and yaw_delta > align_threshold_deg:
            self.get_logger().info(
                f"Step[NAV]: (1/3) rotate to path yaw "
                f"(delta={yaw_delta:.1f}° > {align_threshold_deg:.1f}°)"
            )
            out = self._send_nav_goal_blocking(
                ax, ay, line_yaw, timeout_sec, label="(1/3 align)"
            )
            if out is None or out[0] != GoalStatus.STATUS_SUCCEEDED:
                return out
        else:
            self.get_logger().info(
                f"Step[NAV]: (1/3) skip path-yaw align "
                f"(delta={yaw_delta:.1f}° <= {align_threshold_deg:.1f}°)"
            )

        if dist >= MIN_PATH_DIST_M:
            self.get_logger().info(
                f"Step[NAV]: (2/3) move to B with path yaw={yaw_deg360(line_yaw):.1f}°"
            )
            out = self._send_nav_goal_blocking(
                x, y, line_yaw, timeout_sec, label="(2/3 translate)"
            )
            if out is None or out[0] != GoalStatus.STATUS_SUCCEEDED:
                return out
        else:
            self.get_logger().info("Step[NAV]: (2/3) skip translation (already at B)")

        time.sleep(3.0)

        final_delta = angle_diff_deg(line_yaw, target_yaw)
        if final_delta > FINAL_YAW_SKIP_THRESH_DEG:
            self.get_logger().info(
                f"Step[NAV]: (3/3) final rotation to target yaw={yaw_deg360(target_yaw):.1f}° "
                f"(delta={final_delta:.1f}°)"
            )
            pose = self._get_robot_pose()
            if pose is None:
                self.get_logger().error(
                    f"No pose on {ODOM_TOPIC}, cannot run final rotation"
                )
                return None
            ax, ay, robot_yaw = pose.x, pose.y, pose.yaw
            self.get_logger().info(
                f"Step[NAV]: (3/3) pose A=({ax:.5f}, {ay:.5f}, yaw={yaw_deg360(robot_yaw):.1f}°), "
                f"target_yaw={yaw_deg360(target_yaw):.1f}°, "
                f"{self._pose_time_log_suffix(pose.stamp)}"
            )
            out = self._send_nav_goal_blocking(
                ax, ay, target_yaw, timeout_sec, label="(3/3 rotate)"
            )
            return out

        self.get_logger().info(
            f"Step[NAV]: (3/3) skip final rotation "
            f"(delta={final_delta:.1f}° <= {FINAL_YAW_SKIP_THRESH_DEG:.1f}°)"
        )
        return GoalStatus.STATUS_SUCCEEDED, None

    def _run_yaw_loop(self, stop_idx: int) -> bool:
        """Loop LOOP_COUNT times: stay at (ax, ay), rotate yaw by +YAW_INCREMENT_DEG."""
        self.get_logger().info(
            f"--- Stop {stop_idx}: begin {LOOP_COUNT}x yaw loop (+{YAW_INCREMENT_DEG}°) ---"
        )
        for iteration in range(1, LOOP_COUNT + 1):
            pose = self._get_robot_pose()
            if pose is None:
                self.get_logger().error(
                    f"Stop {stop_idx} loop {iteration}/{LOOP_COUNT}: no pose, abort"
                )
                return False

            ax, ay, yaw = pose.x, pose.y, pose.yaw
            target_yaw = normalize_angle(yaw + math.radians(YAW_INCREMENT_DEG))
            self.get_logger().info(
                f"Stop {stop_idx} loop {iteration}/{LOOP_COUNT}: "
                f"A=({ax:.5f}, {ay:.5f}, yaw={yaw_deg360(yaw):.1f}°) -> "
                f"target yaw={yaw_deg360(target_yaw):.1f}°"
            )

            out = self.navigate_blocking(ax, ay, target_yaw, timeout_sec=NAV_TIMEOUT_SEC)
            if out is None or out[0] != GoalStatus.STATUS_SUCCEEDED:
                status_text = (
                    "None" if out is None else f"{out[0]}({nav_status_to_text(out[0])})"
                )
                self.get_logger().warn(
                    f"Stop {stop_idx} loop {iteration}/{LOOP_COUNT} failed ({status_text})"
                )
                return False

            self.get_logger().info(
                f"Stop {stop_idx} loop {iteration}/{LOOP_COUNT}: succeeded"
            )
            time.sleep(2.0)

        self.get_logger().info(f"--- Stop {stop_idx}: yaw loop complete ---")
        return True

    def _navigate_to_point(
        self, stop_idx: int, x: float, y: float, yaw: float
    ) -> bool:
        self.get_logger().info(
            f"Stop {stop_idx}: navigate to ({x:.5f}, {y:.5f}, yaw={yaw_deg360(yaw):.1f}°)"
        )
        out = self.navigate_blocking(x, y, yaw, timeout_sec=NAV_TIMEOUT_SEC)
        if out is None or out[0] != GoalStatus.STATUS_SUCCEEDED:
            status_text = (
                "None" if out is None else f"{out[0]}({nav_status_to_text(out[0])})"
            )
            self.get_logger().warn(f"Stop {stop_idx} navigation failed ({status_text})")
            return False
        self.get_logger().info(f"Stop {stop_idx}: navigation succeeded")
        return True

    def _run_mission(self) -> None:
        total = len(TEST_POINTS)
        self.get_logger().info(f"=== Workflow test started ({total} waypoints) ===")
        if self._start_message:
            self.get_logger().info(f"Start message: {self._start_message!r}")

        for stop_idx, (x, y, yaw) in enumerate(TEST_POINTS, start=1):
            self.get_logger().info(f"=== Waypoint {stop_idx}/{total} ===")

            if not self._navigate_to_point(stop_idx, x, y, yaw):
                self.get_logger().error(f"Failed to reach waypoint {stop_idx}, aborting")
                break

            if not self._run_yaw_loop(stop_idx):
                self.get_logger().error(
                    f"Yaw loop failed at waypoint {stop_idx}, aborting"
                )
                break

            time.sleep(1.0)

        self.get_logger().info("=== Workflow test complete ===")


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = WorkflowTestNode()

        
        node._pending_start = True

        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
