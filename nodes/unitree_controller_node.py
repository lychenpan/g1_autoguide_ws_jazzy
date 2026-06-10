#!/usr/bin/env python3
"""
Read G1 / Go2 / B2 wireless controller over Unitree native DDS (no ROS msg packages).

Subscribes to rt/wirelesscontroller (type unitree_go.msg.dds_.WirelessController_)
on DDS domain 0 and prints sticks + decoded button states.

Usage (inside cp1-jazzy / cp2-jazzy):
  python3 /workspace/g1_ws_jazzy/nodes/testw.py
  python3 /workspace/g1_ws_jazzy/nodes/testw.py --iface eth0
  python3 /workspace/g1_ws_jazzy/nodes/testw.py --only-on-change
  python3 /workspace/g1_ws_jazzy/nodes/testw.py --combo L1,A,B --combo-hold-secs 2
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass, fields
from typing import Callable, FrozenSet

from sendmsg import DEFAULT_START_MESSAGE, publish_mission_start
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import WirelessController_

DDS_TOPIC = "rt/wirelesscontroller"
DEFAULT_IFACE = "eth0"
DEFAULT_DOMAIN = 0
DEFAULT_COMBO = ("L1", "A", "B")
DEFAULT_COMBO_HOLD_SECS = 2.0


def on_combo_triggered(combo: FrozenSet[str], hold_secs: float) -> None:
    """Called once when the configured 3-button combo is held long enough."""
    combo_str = "+".join(sorted(combo))
    print(f"\n*** COMBO TRIGGERED: {combo_str} held {hold_secs:.1f}s ***\n")
    if {"R1", "L1", "Up"}.issubset(combo):
        publish_mission_start(f"{DEFAULT_START_MESSAGE}:{combo_str}")

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


def parse_combo(text: str) -> frozenset[str]:
    names = [part.strip() for part in text.split(",") if part.strip()]
    if len(names) != 3:
        raise ValueError(f"combo must contain exactly 3 buttons, got {len(names)}: {text!r}")
    valid = {f.name for f in fields(ButtonState)}
    unknown = [name for name in names if name not in valid]
    if unknown:
        raise ValueError(f"unknown button(s): {', '.join(unknown)} (valid: {', '.join(sorted(valid))})")
    return frozenset(names)



    


class ComboHoldDetector:
    """Fire a callback when an exact 3-button combo is held for hold_secs."""

    def __init__(
        self,
        combo: FrozenSet[str],
        hold_secs: float,
        on_trigger: Callable[[FrozenSet[str], float], None],
    ) -> None:
        self.combo = combo
        self.hold_secs = hold_secs
        self.on_trigger = on_trigger
        self._hold_start: float | None = None
        self._triggered = False

    def update(self, buttons: ButtonState) -> None:
        if buttons.pressed_set() != self.combo:
            self._hold_start = None
            self._triggered = False
            return

        now = time.monotonic()
        if self._hold_start is None:
            self._hold_start = now
            return

        if self._triggered:
            return

        elapsed = now - self._hold_start
        if elapsed >= self.hold_secs:
            self._triggered = True
            self.on_trigger(self.combo, elapsed)


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
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.seq = 0
        self.last: RemoteSnapshot | None = None
        self.sub: ChannelSubscriber | None = None
        self.combo_detector: ComboHoldDetector | None = None
        if args.combo is not None:
            self.combo_detector = ComboHoldDetector(
                combo=parse_combo(args.combo),
                hold_secs=args.combo_hold_secs,
                on_trigger=on_combo_triggered,
            )

    def start(self) -> None:
        print(
            f"Initializing DDS domain={self.args.domain} iface={self.args.iface!r} "
            f"topic={DDS_TOPIC}"
        )
        ChannelFactoryInitialize(self.args.domain, self.args.iface)
        self.sub = ChannelSubscriber(DDS_TOPIC, WirelessController_)
        self.sub.Init(self._on_message, self.args.queue_len)
        print("Listening (Ctrl+C to stop)...")
        if self.combo_detector is not None:
            combo_str = "+".join(sorted(self.combo_detector.combo))
            print(
                f"Combo watch: hold {combo_str} together for "
                f"{self.combo_detector.hold_secs:.1f}s to trigger"
            )

    def _on_message(self, msg: WirelessController_) -> None:
        self.seq += 1
        snap = RemoteSnapshot.from_msg(msg)

        if self.combo_detector is not None:
            self.combo_detector.update(snap.buttons)

        if self.args.only_on_change:
            if self.last is not None and self._same(self.last, snap):
                return

        if self.seq % self.args.print_every == 0 or self.args.print_every == 1:
            if self.args.verbose:
                print(snap.format_detail(self.seq))
            else:
                print(snap.format_line(self.seq))

        self.last = snap

    @staticmethod
    def _same(a: RemoteSnapshot, b: RemoteSnapshot) -> bool:
        return (
            a.sticks == b.sticks
            and a.buttons == b.buttons
            and a.keys_raw == b.keys_raw
        )

    def run(self) -> None:
        self.start()
        try:
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print(f"\nStopped after {self.seq} messages.")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Read Unitree wireless controller via unitree_sdk2py DDS."
    )
    p.add_argument(
        "--iface",
        default=DEFAULT_IFACE,
        help=f"Network interface for robot Ethernet (default: {DEFAULT_IFACE})",
    )
    p.add_argument(
        "--domain",
        type=int,
        default=DEFAULT_DOMAIN,
        help=f"DDS domain ID (robot uses {DEFAULT_DOMAIN})",
    )
    p.add_argument(
        "--queue-len",
        type=int,
        default=10,
        dest="queue_len",
        help="Subscriber callback queue length",
    )
    p.add_argument(
        "--print-every",
        type=int,
        default=10,
        help="Print every N-th message (1 = every message)",
    )
    p.add_argument(
        "--only-on-change",
        action="store_true",
        help="Print only when sticks or buttons change",
    )
    p.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="Print multi-line detail including each pressed button",
    )
    p.add_argument(
        "--combo",
        metavar="BTN,BTN,BTN",
        default=None,
        help=(
            "Watch a 3-button combo and trigger after --combo-hold-secs, "
            f"e.g. L1,A,B (common G1 combo)"
        ),
    )
    p.add_argument(
        "--combo-hold-secs",
        type=float,
        default=DEFAULT_COMBO_HOLD_SECS,
        help=f"Seconds all 3 combo buttons must be held (default: {DEFAULT_COMBO_HOLD_SECS})",
    )
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        WirelessReader(args).run()
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
