#!/usr/bin/env python3
"""
Unitree G1 SLAM relocation (unitree_sdk2py only).

Logic mirrors guide2/g1_slam_client.py relocation(); no import from G1SlamClient.
Edit MAP_PATH and other settings below. Initial pose comes from
relocation_init_pose.json (see update_relocation_pos.py).

Usage (CLI):
    python3 update_relocation_pos.py   # capture pose from /unitree/odom (domain 1)
    python3 g1_slam_relocation.py

Usage (library):
    from g1_slam_relocation import start_relocation, start_relocation_with_verification
    status, data = start_relocation()
    status, data = start_relocation_with_verification()  # includes odom recheck + retries
"""

from __future__ import annotations

import json
import os
import subprocess
import time
from pathlib import Path
from typing import Dict, Optional, Tuple

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.rpc.client import Client

# ---------------------------------------------------------------------------
# Configuration (edit here)
# ---------------------------------------------------------------------------
MAP_PATH = "/home/unitree/test.pcd"
NETWORK_INTERFACE = "eth0"
DOMAIN_ID = 0
TIMEOUT_SEC = 10.0

# Post-relocation verification: wait for /unitree/slam_relocation/odom on ROS domain 0.
ODOM_TOPIC = "/unitree/slam_relocation/odom"
ROS_DOMAIN_ID = "0"
ROS_SETUP = os.environ.get("ROS_SETUP", "/opt/ros/foxy/setup.bash")
ODOM_CHECK_SETTLE_SEC = 2.0
ODOM_RATE_SAMPLE_SEC = 5.0
MIN_ODOM_RATE_HZ = 1.0
RETRY_SLEEP_SEC = 10.0
MAX_RESTART_ATTEMPTS = 5

# Initial pose: loaded from tools/relocation_init_pose.json when present.
# Update that file with: python3 tools/update_relocation_pos.py
RELOCATION_POSE_FILE = Path(__file__).resolve().parent / "relocation_init_pose.json"

_DEFAULT_POSE = {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0,
    "q_x": 0.0,
    "q_y": 0.0,
    "q_z": 0.0,
    "q_w": 1.0,
}


def load_relocation_pose(path: Optional[Path] = None) -> Dict[str, float]:
    """Load x, y, z, q_x, q_y, q_z, q_w from JSON (written by update_relocation_pos.py)."""
    pose_file = path or RELOCATION_POSE_FILE
    if not pose_file.is_file():
        return dict(_DEFAULT_POSE)
    with open(pose_file, encoding="utf-8") as f:
        data = json.load(f)
    pose = dict(_DEFAULT_POSE)
    for key in pose:
        if key in data:
            pose[key] = float(data[key])
    return pose


def _current_init_pose() -> Dict[str, float]:
    return load_relocation_pose()

# From g1_slam_client.py / keyDemo.cpp
SLAM_SERVICE_NAME = "slam_operate"
SLAM_API_VERSION = "1.0.0.1"
ROBOT_API_ID_START_RELOCATION_PL = 1804

__all__ = [
    "start_relocation",
    "verify_relocation_odom",
    "start_relocation_with_verification",
    "SlamRelocationClient",
    "load_relocation_pose",
    "MAP_PATH",
    "NETWORK_INTERFACE",
    "RELOCATION_POSE_FILE",
    "SLAM_SERVICE_NAME",
    "SLAM_API_VERSION",
    "ROBOT_API_ID_START_RELOCATION_PL",
]


class SlamRelocationClient(Client):
    """Minimal SLAM RPC client for relocation only (unitree_sdk2py.rpc.client.Client)."""

    def __init__(self) -> None:
        super().__init__(SLAM_SERVICE_NAME, False)

    def init(self) -> None:
        self._SetApiVerson(SLAM_API_VERSION)
        self._RegistApi(ROBOT_API_ID_START_RELOCATION_PL, 0)


def _build_relocation_parameter() -> str:
    """JSON payload for ROBOT_API_ID_START_RELOCATION_PL."""
    pose = _current_init_pose()
    return json.dumps({
        "data": {
            "x": pose["x"],
            "y": pose["y"],
            "z": pose["z"],
            "q_x": pose["q_x"],
            "q_y": pose["q_y"],
            "q_z": pose["q_z"],
            "q_w": pose["q_w"],
            "address": MAP_PATH,
        }
    })


def start_relocation(init_channel: bool = True) -> Tuple[int, Optional[str]]:
    """
    Start SLAM relocation using the hard-coded configuration at the top of this file.

    Args:
        init_channel: If True, call ChannelFactoryInitialize before RPC.
            Set False when another module already initialized the channel.

    Returns:
        (status_code, response_data) — status_code 0 means success.
    """
    if init_channel:
        ChannelFactoryInitialize(DOMAIN_ID, NETWORK_INTERFACE)

    client = SlamRelocationClient()
    client.init()
    client.SetTimeout(TIMEOUT_SEC)

    status_code, data = client._Call(
        ROBOT_API_ID_START_RELOCATION_PL, _build_relocation_parameter()
    )
    return status_code, data


def send_voice_state(state: str) -> None:
    """Send state data to voice."""
    import requests

    BASE = "http://localhost:10011"
    r = requests.post(
        f"{BASE}/api/speak",
        json={"text": state, "wait": True},
        timeout=60)
    r.raise_for_status()
    print(r.json())


def _count_odom_messages(output: str) -> int:
    """Count odom messages in `ros2 topic echo` YAML output."""
    return output.count("\n  stamp:")


def _min_odom_messages(sample_sec: float, min_rate_hz: float) -> int:
    """Require several messages so 1-2 bursts then silence still fail."""
    return max(3, int(min_rate_hz * sample_sec * 0.7))


def verify_relocation_odom(
    topic: str = ODOM_TOPIC,
    sample_sec: float = ODOM_RATE_SAMPLE_SEC,
    min_rate_hz: float = MIN_ODOM_RATE_HZ,
) -> bool:
    """
    Return True when the relocation odom topic keeps publishing during sample_sec.

    A single message (or only 1-2 messages then silence) is treated as failure.
    Uses `ros2 topic echo` and counts messages over the sample window.
    """
    if not Path(ROS_SETUP).is_file():
        print(f"WARNING: ROS setup not found: {ROS_SETUP}")
        return False

    min_messages = _min_odom_messages(sample_sec, min_rate_hz)
    cmd = (
        f'source "{ROS_SETUP}" && '
        f"export ROS_DOMAIN_ID={ROS_DOMAIN_ID} && "
        f"timeout {sample_sec:.1f} ros2 topic echo {topic}"
    )
    print(
        f"Checking odom publish rate: topic={topic} domain={ROS_DOMAIN_ID} "
        f"sample={sample_sec:.1f}s min_rate={min_rate_hz:.1f}Hz "
        f"min_messages={min_messages}"
    )
    try:
        result = subprocess.run(
            ["bash", "-c", cmd],
            capture_output=True,
            text=True,
            timeout=sample_sec + 5.0,
        )
    except (subprocess.TimeoutExpired, OSError) as exc:
        print(f"WARNING: odom check failed to run: {exc}")
        return False

    output = f"{result.stdout}\n{result.stderr}".strip()
    message_count = _count_odom_messages(output)
    measured_rate = message_count / sample_sec if sample_sec > 0 else 0.0
    ok = message_count >= min_messages

    if output:
        preview = output if len(output) <= 400 else output[:400] + "..."
        print(preview)

    print(
        f"Odom check: returncode={result.returncode} "
        f"messages={message_count} measured_rate={measured_rate:.2f}Hz "
        f"required>={min_messages} ok={ok}"
    )
    if not ok:
        print(
            f"WARNING: odom not publishing steadily on {topic} "
            f"({message_count} message(s) in {sample_sec:.1f}s)"
        )
    return ok


def start_relocation_with_verification(
    init_channel: bool = True,
    max_restart_attempts: int = MAX_RESTART_ATTEMPTS,
) -> Tuple[int, Optional[str]]:
    """
    Start relocation, verify /unitree/slam_relocation/odom is publishing, and
    retry relocation up to max_restart_attempts times when verification fails.
    """
    status, data = start_relocation(init_channel=init_channel)
    if status != 0:
        return status, data
    print("relocation and results:")
    print(status, data)

    if ODOM_CHECK_SETTLE_SEC > 0:
        print(f"Waiting {ODOM_CHECK_SETTLE_SEC:.1f}s for odom to start publishing...")
        time.sleep(ODOM_CHECK_SETTLE_SEC)

    if verify_relocation_odom():
        return status, data

    for attempt in range(1, max_restart_attempts + 1):
        message = (
            f"relocation odom not publishing steadily, retrying slam relocation, "
            f"attempt {attempt} of {max_restart_attempts}"
        )
        print(message)
        send_voice_state(message)
        time.sleep(RETRY_SLEEP_SEC)

        status, data = start_relocation(init_channel=False)
        if status != 0:
            print(
                f"WARNING: relocation restart attempt {attempt} failed, "
                f"statusCode={status}, data={data}"
            )
            continue

        if ODOM_CHECK_SETTLE_SEC > 0:
            print(f"Waiting {ODOM_CHECK_SETTLE_SEC:.1f}s for odom to start publishing...")
            time.sleep(ODOM_CHECK_SETTLE_SEC)

        if verify_relocation_odom():
            return status, data

    return -1, "odom verification failed after retries"


def main() -> int:
    import datetime
    now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{now}] start relocation")
    pose = _current_init_pose()
    print("=" * 70)
    print("G1 SLAM Relocation")
    print("=" * 70)
    print(f"  map_path:  {MAP_PATH}")
    print(f"  pose_file: {RELOCATION_POSE_FILE}")
    print(
        f"  pose:      x={pose['x']}, y={pose['y']}, z={pose['z']}, "
        f"q=({pose['q_x']}, {pose['q_y']}, {pose['q_z']}, {pose['q_w']})"
    )
    print(f"  network:   {NETWORK_INTERFACE}")
    print(f"  domain_id: {DOMAIN_ID}")
    print("=" * 70)

    send_voice_state("start relocation G1 slam")
    status, data = start_relocation_with_verification()
    if status == 0:
        print(f"OK: Relocation started and verified with map {MAP_PATH}")
        send_voice_state("relocation G1 slam successfully")
        if data:
            print(f"Response: {data}")
        return 0

    if status == -1:
        print(f"ERROR: Relocation verification failed, data={data}")
        send_voice_state("relocation failed in the end")
        return 1

    print(f"ERROR: Relocation failed, statusCode={status}, data={data}")
    send_voice_state(f"ERROR: Relocation failed, statusCode={status}, data={data}")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
