from __future__ import annotations

import ast
import json
import math
import os
import sys
import threading
import time

_TOOLS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools')
if _TOOLS_DIR not in sys.path:
    sys.path.insert(0, os.path.normpath(_TOOLS_DIR))

from unitree_sdk2py.core.channel import ChannelFactoryInitialize  # noqa: E402

ChannelFactoryInitialize(0, os.environ.get('UNITREE_NET_IFACE', 'eth0'))
os.environ.setdefault('ROS_DOMAIN_ID', '1')

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Empty
from showroom_control import next_page, play_video
from tts_player import RemoteTTSPlayer

_DATA_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'data')
)
_POINTS_FILE = os.path.join(_DATA_DIR, 'points.txt')
_SHOWROOM_FILE = os.path.join(_DATA_DIR, 'showroom_speak.json')
MISSION_START_TOPIC = os.environ.get('SHOWROOM_MISSION_START_TOPIC', '/showroom_mission/start')
EXPECTED_STOPS = 8
EXPECTED_SLOT_COUNT = 5
FIXED_SPEAK_STOPS = 3  # stops 1–3: fixedspeaktext; 4–8: PPT slots

# Global test flag: True = each TTS line speaks only the first 15 characters.
TEST = True
if "SHOWROOM_TEST" in os.environ:
    TEST = os.environ["SHOWROOM_TEST"].lower() in ("1", "true", "yes")
TEST_TTS_MAX_CHARS = 15


def text_for_tts(text: str) -> str:
    if TEST:
        return text[:TEST_TTS_MAX_CHARS]
    return text


def load_goals(path: str) -> list[tuple[float, float, float]]:
    goals = []
    with open(path, 'r', encoding='utf-8') as f:
        for line_no, line in enumerate(f, start=1):
            code = line.split('#', 1)[0].strip().rstrip(',')
            if not code:
                continue
            try:
                point = ast.literal_eval(code)
            except (SyntaxError, ValueError) as exc:
                raise ValueError(f"Invalid point at {path}:{line_no}: {line!r}") from exc
            if not isinstance(point, (tuple, list)) or len(point) != 3:
                raise ValueError(f"Expected (x, y, yaw) at {path}:{line_no}: {line!r}")
            goals.append((float(point[0]), float(point[1]), float(point[2])))
    if not goals:
        raise ValueError(f"No goals found in {path}")
    return goals


def slot_speak_texts(slot_row: dict) -> list[str]:
    slide_count = int(slot_row["slideCount"])
    speak_by_page = slot_row.get("speakByPage") or {}
    return [str(speak_by_page.get(str(i), "")) for i in range(slide_count)]


def load_voice_text_lists(path: str) -> list[list[str]]:
    with open(path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    voice_lists: list[list[str]] = []

    fixed = data.get("fixedspeaktext") or []
    if not isinstance(fixed, list):
        raise ValueError(f"{path}: fixedspeaktext must be a list")
    for text in fixed:
        voice_lists.append([str(text)])

    slots = data.get("slots") or []
    if not isinstance(slots, list):
        raise ValueError(f"{path}: slots must be a list")
    slots_sorted = sorted(slots, key=lambda row: int(row["slot"]))
    if len(slots_sorted) != EXPECTED_SLOT_COUNT:
        raise ValueError(
            f"{path}: expected {EXPECTED_SLOT_COUNT} slots, got {len(slots_sorted)}"
        )

    for slot_row in slots_sorted:
        voice_lists.append(slot_speak_texts(slot_row))

    if len(voice_lists) != EXPECTED_STOPS:
        raise ValueError(
            f"{path}: expected {EXPECTED_STOPS} voice lists "
            f"({len(fixed)} fixed + {EXPECTED_SLOT_COUNT} slots), got {len(voice_lists)}"
        )
    return voice_lists


def build_mission_steps(
    points_path: str = _POINTS_FILE,
    showroom_path: str = _SHOWROOM_FILE,
) -> list[tuple[tuple[float, float, float], list[str]]]:
    points = load_goals(points_path)
    voice_lists = load_voice_text_lists(showroom_path)
    if len(points) != len(voice_lists):
        raise ValueError(
            f"points ({len(points)} in {points_path}) != "
            f"voice stops ({len(voice_lists)} in {showroom_path})"
        )
    return list(zip(points, voice_lists))


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


class ShowroomWorkflowNode(Node):
    """ROS 2 node: wait for start topic, then run 8 navigate + TTS steps."""

    def __init__(
        self,
        mission_steps: list[tuple[tuple[float, float, float], list[str]]],
        tts: RemoteTTSPlayer,
    ):
        super().__init__("showroom_workflow")
        self._mission_steps = mission_steps
        self._tts = tts
        self._running = False
        self._pending_start = False
        self._mission_thread: threading.Thread | None = None
        self._nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.create_subscription(Empty, MISSION_START_TOPIC, self._on_start_request, 10)
        self.create_timer(0.2, self._mission_timer_cb)
        self.get_logger().info(
            f"Loaded {len(mission_steps)} stops from {_SHOWROOM_FILE} + {_POINTS_FILE}"
        )
        self.get_logger().info(
            f"Publish std_msgs/Empty to {MISSION_START_TOPIC} to begin mission"
        )

    def _on_start_request(self, _msg: Empty) -> None:
        if self._running:
            self.get_logger().warn("Mission already running, ignoring start request")
            return
        self.get_logger().info("Start request received")
        self._pending_start = True

    def _mission_timer_cb(self) -> None:
        if not self._pending_start or self._running:
            return
        self._pending_start = False
        self._running = True
        self._mission_thread = threading.Thread(
            target=self._run_mission_wrapper,
            name="showroom-mission-thread",
            daemon=True,
        )
        self._mission_thread.start()

    def _run_mission_wrapper(self) -> None:
        try:
            self._run_mission()
        except Exception as exc:
            self.get_logger().exception(f"Mission crashed: {exc}")
        finally:
            self._running = False

    def navigate_blocking(
        self, x: float, y: float, yaw: float, timeout_sec: float = 300.0
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

        self.get_logger().info(
            f"Step[NAV]: sending goal x={x:.5f}, y={y:.5f}, yaw={yaw:.5f}"
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
            f"Step[NAV]: status={wrapped.status}({nav_status_to_text(wrapped.status)})"
        )
        return wrapped.status, wrapped.result

    def _advance_ppt_slide(self, stop_idx: int, part_idx: int) -> None:
        n = stop_idx - FIXED_SPEAK_STOPS
        self.get_logger().info(
            f"Step[PPT]: next_page after stop {stop_idx} part {part_idx}, n={n}"
        )
        ## TODO, for kinds of remote control or call, if some exceptions occurs, let the 
        ## robot to speak out the error message.
        try:
            result = next_page(n=n)
        except Exception as exc:
            self.get_logger().warn(f"Step[PPT]: next_page failed: {exc}")
            return
        if result.ok:
            self.get_logger().info(f"Step[PPT]: next_page OK (HTTP {result.status_code})")
        else:
            self.get_logger().warn(
                f"Step[PPT]: next_page HTTP {result.status_code}: {result.body[:200]}"
            )

    def _play_voice_list(self, stop_idx: int, texts: list[str]) -> None:
        is_ppt_slot = stop_idx > FIXED_SPEAK_STOPS
        for part_idx, text in enumerate(texts, start=1):
            text = text.strip()
            if not text:
                self.get_logger().warn(
                    f"Step[TTS]: stop {stop_idx} part {part_idx}/{len(texts)} empty, sleep 2s"
                )
                time.sleep(2.0)
            else:
                speak_text = text_for_tts(text)
                self.get_logger().info(
                    f"Step[TTS]: stop {stop_idx} part {part_idx}/{len(texts)}, "
                    f"chars={len(text)}"
                    + (
                        f", TEST mode speak first {len(speak_text)} chars: {speak_text!r}"
                        if TEST
                        else ""
                    )
                )
                self._tts.playtext(speak_text)
                self._tts.wait_done()
            if is_ppt_slot:
                self._advance_ppt_slide(stop_idx, part_idx)

    def _run_mission(self) -> None:
        total = len(self._mission_steps)
        self.get_logger().info(f"=== Showroom mission started ({total} stops) ===")
        if TEST:
            self.get_logger().info(
                f"TEST mode: TTS limited to first {TEST_TTS_MAX_CHARS} characters per line"
            )

        for stop_idx, ((x, y, yaw), voice_texts) in enumerate(self._mission_steps, start=1):
            self.get_logger().info(f"--- Stop {stop_idx}/{total} begin ---")
            self.get_logger().info(
                f"Step[NAV]: navigate to ({x:.5f}, {y:.5f}, {yaw:.5f}), "
                f"TTS parts={len(voice_texts)}"
            )

            out = self.navigate_blocking(x, y, yaw, timeout_sec=300.0)
            if out is None or out[0] != GoalStatus.STATUS_SUCCEEDED:
                status_text = "None" if out is None else f"{out[0]}({nav_status_to_text(out[0])})"
                self.get_logger().warn(
                    f"Step[NAV]: stop {stop_idx} failed ({status_text}), skip TTS"
                )
                continue

            self.get_logger().info(f"Step[NAV]: stop {stop_idx} succeeded")
            # time.sleep(1.0)
            self._play_voice_list(stop_idx, voice_texts)
            self.get_logger().info(f"--- Stop {stop_idx}/{total} end ---")

            ## for the first point, play video and pause untile the video is over.
            if stop_idx == 1:
                play_video()
                if TEST:
                    time.sleep(10)
                    play_video(command="PauseVideo")
                else:
                    time.sleep(5*60+30+3)  # 5:30 seconds
            else:
                time.sleep(1.0)

        self.get_logger().info("=== Showroom mission complete ===")


def main():
    tts = RemoteTTSPlayer()
    rclpy.init()
    node = None
    try:
        mission_steps = build_mission_steps()
        node = ShowroomWorkflowNode(mission_steps, tts)

        if os.environ.get("SHOWROOM_AUTO_START", "").lower() in ("1", "true", "yes"):
            node.get_logger().info("SHOWROOM_AUTO_START set, starting mission immediately")
            node._pending_start = True

        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        tts.stop()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
