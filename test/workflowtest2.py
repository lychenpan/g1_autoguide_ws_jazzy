#!/usr/bin/env python3
"""Voice test: play TTS lines from showroom_speak.json when mission start is received.

Set SHOWROOM_AUTO_START=1 to run immediately on launch.
"""

from __future__ import annotations

import json
import os
import sys
import threading
import time

_TOOLS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools")
if _TOOLS_DIR not in sys.path:
    sys.path.insert(0, os.path.normpath(_TOOLS_DIR))

from unitree_sdk2py.core.channel import ChannelFactoryInitialize  # noqa: E402

ChannelFactoryInitialize(0, os.environ.get("UNITREE_NET_IFACE", "eth0"))
os.environ.setdefault("ROS_DOMAIN_ID", "1")

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from showroom_control import next_page
from tts_player import RemoteTTSPlayer

_DATA_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data")
)
_SHOWROOM_FILE = os.path.join(_DATA_DIR, "showroom_speak.json")
MISSION_START_TOPIC = os.environ.get(
    "SHOWROOM_MISSION_START_TOPIC", "/showroom_mission/start"
)
EXPECTED_STOPS = 8
EXPECTED_SLOT_COUNT = 5
FIXED_SPEAK_STOPS = 3  # stops 1–3: fixedspeaktext; 4–8: PPT slots

TEST = True
if "SHOWROOM_TEST" in os.environ:
    TEST = os.environ["SHOWROOM_TEST"].lower() in ("1", "true", "yes")
TEST_TTS_MAX_CHARS = 15


def text_for_tts(text: str) -> str:
    if TEST:
        return text[:TEST_TTS_MAX_CHARS]
    return text


def slot_speak_texts(slot_row: dict) -> list[str]:
    slide_count = int(slot_row["slideCount"])
    speak_by_page = slot_row.get("speakByPage") or {}
    return [str(speak_by_page.get(str(i), "")) for i in range(slide_count)]


def load_voice_text_lists(path: str) -> list[list[str]]:
    with open(path, "r", encoding="utf-8") as f:
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


class VoiceTestNode(Node):
    """ROS 2 node: wait for start topic, then play TTS for each voice stop."""

    def __init__(self, voice_lists: list[list[str]], tts: RemoteTTSPlayer):
        super().__init__("workflow_voice_test")
        self._voice_lists = voice_lists
        self._tts = tts
        self._running = False
        self._pending_start = False
        self._start_message = ""
        self._mission_thread: threading.Thread | None = None
        self.create_subscription(String, MISSION_START_TOPIC, self._on_start_request, 10)
        self.create_timer(0.2, self._mission_timer_cb)
        self.get_logger().info(
            f"Voice test loaded: {len(voice_lists)} stops from {_SHOWROOM_FILE}"
        )
        self.get_logger().info(
            f"Publish std_msgs/String to {MISSION_START_TOPIC} to begin test"
        )

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
            name="workflow-voice-test-thread",
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

    def _advance_ppt_slide(self, stop_idx: int, part_idx: int) -> None:
        n = stop_idx - FIXED_SPEAK_STOPS
        self.get_logger().info(
            f"Step[PPT]: next_page after stop {stop_idx} part {part_idx}, n={n}"
        )
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
        total = len(self._voice_lists)
        self.get_logger().info(f"=== Voice test started ({total} stops) ===")
        if self._start_message:
            self.get_logger().info(f"Start message: {self._start_message!r}")
        if TEST:
            self.get_logger().info(
                f"TEST mode: TTS limited to first {TEST_TTS_MAX_CHARS} characters per line"
            )

        for stop_idx, voice_texts in enumerate(self._voice_lists, start=1):
            self.get_logger().info(f"--- Stop {stop_idx}/{total} begin ---")
            self.get_logger().info(f"Step[TTS]: parts={len(voice_texts)}")
            self._play_voice_list(stop_idx, voice_texts)
            self.get_logger().info(f"--- Stop {stop_idx}/{total} end ---")
            time.sleep(1.0)

        self.get_logger().info("=== Voice test complete ===")


def main() -> None:
    tts = RemoteTTSPlayer()
    rclpy.init()
    node = None
    try:
        voice_lists = load_voice_text_lists(_SHOWROOM_FILE)
        node = VoiceTestNode(voice_lists, tts)
        node._pending_start = True

        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        tts.stop()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
