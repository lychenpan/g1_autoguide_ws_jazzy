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
    from g1_slam_relocation import start_relocation
    status, data = start_relocation()
"""

from __future__ import annotations

import json
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


def main() -> int:
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

    status, data = start_relocation()

    if status == 0:
        print(f"OK: Relocation started with map {MAP_PATH}")
        if data:
            print(f"Response: {data}")
        return 0

    print(f"ERROR: Relocation failed, statusCode={status}, data={data}")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
