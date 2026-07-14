#!/usr/bin/env python3
"""
Read G1 / Go2 / B2 wireless controller over Unitree native DDS (no ROS msg packages).

Subscribes to rt/wirelesscontroller (type unitree_go.msg.dds_.WirelessController_)
on DDS domain 0 and prints sticks + decoded button states.

Usage (inside cp1-jazzy / cp2-jazzy):
  python3 /workspace/g1_ws_jazzy/nodes/unitree_controller_node.py
"""

from __future__ import annotations

import signal
import logging
import sys
import threading
import time
from dataclasses import dataclass, fields
from typing import Callable, FrozenSet

from tools import (
    DEFAULT_START_MESSAGE,
    publish_mission_start,
    start_nav2_restart_async,
    start_wifi_switch_async,
    stop_docker_container,
)
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import WirelessController_

DDS_TOPIC = "rt/wirelesscontroller"
IFACE = "eth0"
DOMAIN = 0
QUEUE_LEN = 10
PRINT_EVERY = 10
ONLY_ON_CHANGE = False
VERBOSE = False
BUTTON_COUNT = 3
HOLD_SECS = 1.0

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger(__name__)

def send_voice_state(state: str) -> None:
    """Send state data to voice."""
    import requests

    BASE = "http://localhost:10011"
    logger.info("send voice state: %s", state)
    r = requests.post(
        f"{BASE}/api/speak",
        json={"text": state, "wait": True},
        timeout=60)
    r.raise_for_status()
    logger.info("voice response: %s", r.json())

def on_combo_triggered(combo: FrozenSet[str], hold_secs: float) -> None:
    """Called once when any 3-button combo is held long enough."""
    combo_str = "   ".join(sorted(combo))
    logger.info("COMBO TRIGGERED: %s held %.1fs", combo_str, hold_secs)
    send_voice_state(f"Combo triggered: {combo_str} ")
    logger.info("sendvoicestate")
    if {"R1", "L1", "Up"}.issubset(combo):
        publish_mission_start(f"------------{DEFAULT_START_MESSAGE}:{combo_str}")

    if {"R1", "L1", "Down"}.issubset(combo):
        start_nav2_restart_async()
    
    if {"R1", "L1", "Left"}.issubset(combo):
        stop_docker_container()

    if {"R1", "L1", "A"}.issubset(combo):
        start_wifi_switch_async()


@dataclass
class StickState:
    lx: float = 0.0
    ly: float = 0.0
    rx: float = 0.0
    ry: float = 0.0


@dataclass
class ButtonState:
    """Decoded from WirelessController.keys (uint16 xKeySwitchUnion bitfield)."""

    L1: int = 0
    L2: int = 0
    R1: int = 0
    R2: int = 0
    A: int = 0
    B: int = 0
    X: int = 0
    Y: int = 0
    Up: int = 0
    Down: int = 0
    Left: int = 0
    Right: int = 0
    Select: int = 0
    Start: int = 0
    F1: int = 0
    F3: int = 0

    @classmethod
    def from_keys(cls, keys: int) -> ButtonState:
        """Decode keys as xKeySwitchUnion uint16 (advanced_gamepad.hpp)."""
        k = keys & 0xFFFF
        return cls(
            R1=(k >> 0) & 1,
            L1=(k >> 1) & 1,
            Start=(k >> 2) & 1,
            Select=(k >> 3) & 1,
            R2=(k >> 4) & 1,
            L2=(k >> 5) & 1,
            F1=(k >> 6) & 1,
            F3=(k >> 7) & 1,
            A=(k >> 8) & 1,
            B=(k >> 9) & 1,
            X=(k >> 10) & 1,
            Y=(k >> 11) & 1,
            Up=(k >> 12) & 1,
            Right=(k >> 13) & 1,
            Down=(k >> 14) & 1,
            Left=(k >> 15) & 1,
        )

    def pressed_names(self) -> list[str]:
        return [f.name for f in fields(self) if getattr(self, f.name)]

    def pressed_set(self) -> frozenset[str]:
        return frozenset(self.pressed_names())


class TripleButtonHoldDetector:
    """Fire a callback when exactly button_count buttons are held for hold_secs."""

    def __init__(
        self,
        button_count: int,
        hold_secs: float,
        on_trigger: Callable[[FrozenSet[str], float], None],
    ) -> None:
        self.button_count = button_count
        self.hold_secs = hold_secs
        self.on_trigger = on_trigger
        self._hold_start: float | None = None
        self._active_combo: FrozenSet[str] | None = None
        self._triggered_combo: FrozenSet[str] | None = None

    def update(self, buttons: ButtonState) -> None:
        pressed = buttons.pressed_set()

        if len(pressed) != self.button_count:
            self._hold_start = None
            self._active_combo = None
            self._triggered_combo = None
            return

        if pressed != self._active_combo:
            self._active_combo = pressed
            self._hold_start = None
            self._triggered_combo = None

        now = time.monotonic()
        if self._hold_start is None:
            self._hold_start = now
            return

        if self._triggered_combo == pressed:
            return

        elapsed = now - self._hold_start
        if elapsed >= self.hold_secs:
            self._triggered_combo = pressed
            self.on_trigger(pressed, elapsed)


@dataclass
class RemoteSnapshot:
    sticks: StickState
    buttons: ButtonState
    keys_raw: int

    @classmethod
    def from_msg(cls, msg: WirelessController_) -> RemoteSnapshot:
        return cls(
            sticks=StickState(msg.lx, msg.ly, msg.rx, msg.ry),
            buttons=ButtonState.from_keys(int(msg.keys)),
            keys_raw=int(msg.keys),
        )

    def format_line(self, seq: int) -> str:
        s = self.sticks
        pressed = self.buttons.pressed_names()
        btn_str = ",".join(pressed) if pressed else "(none)"
        return (
            f"#{seq} "
            f"lx={s.lx:+.3f} ly={s.ly:+.3f} rx={s.rx:+.3f} ry={s.ry:+.3f} "
            f"keys=0x{self.keys_raw:04x} buttons=[{btn_str}]"
        )

    def format_detail(self, seq: int) -> str:
        lines = [self.format_line(seq), "  sticks: " + str(self.sticks)]
        lines.append("  buttons:")
        for f in fields(self.buttons):
            v = getattr(self.buttons, f.name)
            if v:
                lines.append(f"    {f.name}=1")
        if not self.buttons.pressed_names():
            lines.append("    (all released)")
        return "\n".join(lines)


class WirelessReader:
    def __init__(self) -> None:
        self.seq = 0
        self.last: RemoteSnapshot | None = None
        self.sub: ChannelSubscriber | None = None
        self._stop_event = threading.Event()
        self.combo_detector = TripleButtonHoldDetector(
            button_count=BUTTON_COUNT,
            hold_secs=HOLD_SECS,
            on_trigger=on_combo_triggered,
        )

    def start(self) -> None:
        logger.info(
            "Initializing DDS domain=%s iface=%r topic=%s",
            DOMAIN,
            IFACE,
            DDS_TOPIC,
        )
        ChannelFactoryInitialize(DOMAIN, IFACE)
        self.sub = ChannelSubscriber(DDS_TOPIC, WirelessController_)
        self.sub.Init(self._on_message, QUEUE_LEN)
        logger.info("Listening (Ctrl+C to stop)...")
        logger.info(
            "Combo watch: hold any %s buttons together for %.1fs to trigger",
            self.combo_detector.button_count,
            self.combo_detector.hold_secs,
        )

    def _on_message(self, msg: WirelessController_) -> None:
        self.seq += 1
        snap = RemoteSnapshot.from_msg(msg)

        self.combo_detector.update(snap.buttons)

        if ONLY_ON_CHANGE:
            if self.last is not None and self._same(self.last, snap):
                return

        # if self.seq % PRINT_EVERY == 0 or PRINT_EVERY == 1:
        
        #     if VERBOSE:
        #         logger.info("%s", snap.format_detail(self.seq))
        #     else:
        #         logger.info("%s", snap.format_line(self.seq))

        self.last = snap

    @staticmethod
    def _same(a: RemoteSnapshot, b: RemoteSnapshot) -> bool:
        return (
            a.sticks == b.sticks
            and a.buttons == b.buttons
            and a.keys_raw == b.keys_raw
        )

    def _request_stop(self, *_args) -> None:
        self._stop_event.set()

    def run(self) -> None:
        self.start()
        signal.signal(signal.SIGINT, self._request_stop)
        signal.signal(signal.SIGTERM, self._request_stop)
        try:
            # DDS delivers controller data in a background callback thread.
            # Block the main thread until Ctrl+C or systemd sends SIGTERM.
            self._stop_event.wait()
        finally:
            if self.sub is not None:
                self.sub.Close()
            logger.info("Stopped after %s messages.", self.seq)


def main() -> int:
    try:
        WirelessReader().run()
    except Exception as e:
        logger.exception("Error: %s", e)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
