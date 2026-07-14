#!/usr/bin/env python3
"""ROS helpers and Docker Nav2 restart/verification utilities."""

from __future__ import annotations

import logging
import os
import subprocess
import threading
import time

os.environ["ROS_DOMAIN_ID"] = "1"

import rclpy
from std_msgs.msg import String

logger = logging.getLogger(__name__)

MISSION_START_TOPIC = "/showroom_mission/start"
DEFAULT_START_MESSAGE = "start"

DOCKER_BIN = "docker"
DOCKER_CONTAINER = "cp2-jazzy"
DOCKER_WS_DIR = "/workspace/g1_ws_jazzy"
DOCKER_BASHRC = "/root/.bashrc"
NAV2_ROS_DOMAIN_ID = "1"
NAV2_VERIFY_TIMEOUT_SEC = 180
NAV2_VERIFY_INTERVAL_SEC = 5
NAV2_REQUIRED_TOPICS = (
    "/unitree/odom",
    "/utlidar/pcl2",
    "/map",
    "/cmd_vel",
    "/lifecycle_manager_navigation/managed_nodes_activated",
)
TTS_SPEAK_URL = "http://localhost:10011/api/speak"
NAV2_RESTART_SPEAK_TEXT = "Restarting navigation stack"
NAV2_READY_SPEAK_TEXT = "Nav2 stack ready"
NAV2_FAIL_SPEAK_TEXT = "Nav2 stack failed to start"

WIFI_CANDIDATES = ("展厅专用", "BIFNC Guest")
NMCLI_BIN = "nmcli"
# connection up needs root; uses passwordless sudo (see tools/sudoers-g1-wifi-nmcli)
NMCLI_SUDO = ("sudo", "-n", "/usr/bin/nmcli")


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


def _docker_inspect_running(container: str) -> bool:
    result = subprocess.run(
        [DOCKER_BIN, "inspect", "-f", "{{.State.Running}}", container],
        capture_output=True,
        text=True,
        check=False,
    )
    return result.returncode == 0 and result.stdout.strip() == "true"


def stop_docker_container(container: str = DOCKER_CONTAINER) -> bool:
    """Stop the Docker container (default: cp2-jazzy)."""
    if subprocess.run(["which", DOCKER_BIN], capture_output=True, check=False).returncode != 0:
        logger.error("docker command not found: %s", DOCKER_BIN)
        return False

    inspect = subprocess.run(
        [DOCKER_BIN, "inspect", container],
        capture_output=True,
        check=False,
    )
    if inspect.returncode != 0:
        logger.error("Docker container not found: %s", container)
        return False

    logger.info("Stopping Docker container: %s", container)
    stop = subprocess.run(
        [DOCKER_BIN, "stop", container],
        capture_output=True,
        text=True,
        check=False,
    )
    if stop.returncode != 0:
        logger.error("Failed to stop Docker container %s: %s", container, stop.stderr.strip())
        return False

    logger.info("Docker container stopped: %s", container)
    speak_tts("docker container of navigation stopped")
    return True


def restart_docker_nav2(
    container: str = DOCKER_CONTAINER,
    ws_dir: str = DOCKER_WS_DIR,
    bashrc: str = DOCKER_BASHRC,
) -> bool:
    """Restart cp2-jazzy and launch startnav2.sh inside it."""
    if subprocess.run(["which", DOCKER_BIN], capture_output=True, check=False).returncode != 0:
        logger.error("docker command not found: %s", DOCKER_BIN)
        return False

    inspect = subprocess.run(
        [DOCKER_BIN, "inspect", container],
        capture_output=True,
        check=False,
    )
    if inspect.returncode != 0:
        logger.error("Docker container not found: %s", container)
        return False

    logger.info("Restarting Docker container: %s", container)
    restart = subprocess.run(
        [DOCKER_BIN, "restart", container],
        capture_output=True,
        text=True,
        check=False,
    )
    if restart.returncode != 0:
        logger.error("Failed to restart Docker container %s: %s", container, restart.stderr.strip())
        return False

    running = False
    for _ in range(30):
        if _docker_inspect_running(container):
            running = True
            break
        time.sleep(1)

    if not running:
        logger.error("Docker container did not reach running state: %s", container)
        return False

    launch_cmd = (
        f'PS1=1 source "{bashrc}" && cd "{ws_dir}" && bash startnav2.sh'
    )
    logger.info("Launching Nav2 in %s: %s/startnav2.sh", container, ws_dir)
    launch = subprocess.run(
        [DOCKER_BIN, "exec", "-d", container, "bash", "-lc", launch_cmd],
        capture_output=True,
        text=True,
        check=False,
    )
    if launch.returncode != 0:
        logger.error("Failed to exec startnav2.sh in %s: %s", container, launch.stderr.strip())
        return False

    logger.info("Nav2 launch started inside Docker container: %s", container)
    return True


def docker_nav2_topic_list(
    container: str = DOCKER_CONTAINER,
    bashrc: str = DOCKER_BASHRC,
    ros_domain_id: str = NAV2_ROS_DOMAIN_ID,
) -> str:
    """Return ros2 topic list output from inside the Docker container."""
    topic_cmd = (
        f'PS1=1 source "{bashrc}" && '
        f"export ROS_DOMAIN_ID={ros_domain_id} && ros2 topic list"
    )
    result = subprocess.run(
        [DOCKER_BIN, "exec", container, "bash", "-lc", topic_cmd],
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        return ""
    return result.stdout


def verify_docker_nav2(
    container: str = DOCKER_CONTAINER,
    required_topics: tuple[str, ...] = NAV2_REQUIRED_TOPICS,
    timeout_sec: int = NAV2_VERIFY_TIMEOUT_SEC,
    interval_sec: int = NAV2_VERIFY_INTERVAL_SEC,
    ros_domain_id: str = NAV2_ROS_DOMAIN_ID,
    bashrc: str = DOCKER_BASHRC,
) -> bool:
    """Poll until all required Nav2 topics are present (same logic as g1-tools-start.sh)."""
    deadline = time.monotonic() + timeout_sec
    logger.info(
        "Waiting for Nav2 on ROS domain %s (timeout %ss)...",
        ros_domain_id,
        timeout_sec,
    )
    logger.info("Required topics: %s", " ".join(required_topics))

    while time.monotonic() < deadline:
        topics_list = docker_nav2_topic_list(container, bashrc, ros_domain_id)
        if topics_list:
            topic_lines = {line.strip() for line in topics_list.splitlines() if line.strip()}
            missing = [topic for topic in required_topics if topic not in topic_lines]
            if not missing:
                logger.info("Nav2 verification OK; all required topics present")
                return True
            logger.info("Nav2 topics still missing:%s", " ".join(missing))
        else:
            logger.info("Nav2 topic list empty or unavailable")

        time.sleep(interval_sec)

    logger.warning("Nav2 verification timed out after %ss", timeout_sec)
    return False


def restart_and_verify_docker_nav2() -> bool:
    """Restart cp2-jazzy, launch Nav2, and wait until topic checks pass."""
    if not restart_docker_nav2():
        return False
    return verify_docker_nav2()


def speak_tts(text: str) -> None:
    """Send text to the local TTS web server."""
    import requests

    logger.info("TTS speak: %s", text)
    try:
        response = requests.post(
            TTS_SPEAK_URL,
            json={"text": text, "wait": True},
            timeout=60,
        )
        response.raise_for_status()
        logger.info("TTS response: %s", response.json())
    except Exception as exc:
        logger.warning("TTS speak request failed: %s", exc)


def handle_nav2_restart() -> None:
    """Restart cp2-jazzy, verify Nav2 topics, and announce result via TTS."""
    speak_tts(NAV2_RESTART_SPEAK_TEXT)
    if restart_and_verify_docker_nav2():
        speak_tts(NAV2_READY_SPEAK_TEXT)
        logger.info("Docker Nav2 restart and verification succeeded")
        return

    speak_tts(NAV2_FAIL_SPEAK_TEXT)
    logger.warning("Docker Nav2 restart or verification failed")


def start_nav2_restart_async() -> threading.Thread:
    """Run handle_nav2_restart in a background thread."""
    thread = threading.Thread(
        target=handle_nav2_restart,
        name="nav2-restart",
        daemon=True,
    )
    thread.start()
    return thread


def _nmcli(*args: str, timeout: float = 60) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [NMCLI_BIN, *args],
        capture_output=True,
        text=True,
        check=False,
        timeout=timeout,
    )


def scan_wifi_signals(
    ssids: tuple[str, ...] = WIFI_CANDIDATES,
    rescan: bool = True,
) -> dict[str, int]:
    """Return best SIGNAL (0-100) for each requested SSID from nmcli scan."""
    rescan_arg = "yes" if rescan else "no"
    result = _nmcli("-t", "-f", "SSID,SIGNAL", "device", "wifi", "list", "--rescan", rescan_arg)
    if result.returncode != 0:
        logger.error("nmcli wifi list failed: %s", result.stderr.strip())
        return {}

    wanted = set(ssids)
    best: dict[str, int] = {}
    for line in result.stdout.splitlines():
        if not line or ":" not in line:
            continue
        ssid, signal_str = line.rsplit(":", 1)
        if ssid not in wanted:
            continue
        try:
            signal = int(signal_str)
        except ValueError:
            continue
        if signal > best.get(ssid, -1):
            best[ssid] = signal

    logger.info("WiFi scan signals: %s", best)
    return best


def pick_better_wifi(
    signals: dict[str, int],
    ssids: tuple[str, ...] = WIFI_CANDIDATES,
) -> str | None:
    """Pick SSID with the highest signal; None if none are visible."""
    visible = [(ssid, signals[ssid]) for ssid in ssids if ssid in signals]
    if not visible:
        return None
    visible.sort(key=lambda item: item[1], reverse=True)
    return visible[0][0]


def connect_wifi(ssid: str) -> bool:
    """Bring up a saved NetworkManager connection by name (needs passwordless sudo)."""
    logger.info("Connecting WiFi: %s", ssid)
    try:
        result = subprocess.run(
            [*NMCLI_SUDO, "connection", "up", ssid],
            capture_output=True,
            text=True,
            check=False,
            timeout=90,
        )
    except subprocess.TimeoutExpired:
        logger.error("Timed out connecting WiFi: %s", ssid)
        return False

    if result.returncode != 0:
        err = (result.stderr or result.stdout or "").strip()
        logger.error("Failed to connect WiFi %s: %s", ssid, err)
        if "password is required" in err.lower() or "a password is required" in err.lower():
            logger.error(
                "Passwordless sudo for nmcli is not configured. "
                "Install tools/sudoers-g1-wifi-nmcli into /etc/sudoers.d/"
            )
        return False
    logger.info("Connected WiFi: %s", ssid)
    return True


def active_wifi_connection() -> str | None:
    """Return the active 802-11-wireless connection name, if any."""
    result = _nmcli("-t", "-f", "NAME,TYPE", "connection", "show", "--active")
    if result.returncode != 0:
        return None
    for line in result.stdout.splitlines():
        if not line:
            continue
        name, _, typ = line.partition(":")
        if typ == "802-11-wireless":
            return name
    return None


def switch_to_better_wifi(
    ssids: tuple[str, ...] = WIFI_CANDIDATES,
) -> str | None:
    """Scan candidate SSIDs, connect to the stronger one, and announce via TTS.

    Returns the chosen SSID on success, or None on failure.
    """
    signals = scan_wifi_signals(ssids=ssids, rescan=True)
    best = pick_better_wifi(signals, ssids=ssids)
    if best is None:
        speak_tts("No candidate WiFi networks found")
        logger.warning("No candidate WiFi networks found among %s", ssids)
        return None

    other = [s for s in ssids if s != best and s in signals]
    if other:
        speak_text = (
            f"{best} has better signal, {signals[best]} percent, "
            f"versus {other[0]} at {signals[other[0]]} percent"
        )
    else:
        speak_text = f"{best} has better signal, {signals[best]} percent"
    speak_tts(speak_text)

    current = active_wifi_connection()
    if current == best:
        logger.info("Already connected to stronger WiFi: %s", best)
        speak_tts(f"Already connected to {best}")
        return best

    if not connect_wifi(best):
        speak_tts(f"Failed to connect to {best}")
        return None

    speak_tts(f"Connected to {best} successfully")
    return best


def handle_wifi_switch() -> None:
    """Scan, switch to the stronger of the two showroom WiFi networks, speak result."""
    chosen = switch_to_better_wifi()
    if chosen:
        logger.info("WiFi switch chose: %s", chosen)
    else:
        logger.warning("WiFi switch failed")


def start_wifi_switch_async() -> threading.Thread:
    """Run handle_wifi_switch in a background thread."""
    thread = threading.Thread(
        target=handle_wifi_switch,
        name="wifi-switch",
        daemon=True,
    )
    thread.start()
    return thread


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    )
    ok = switch_to_better_wifi() is not None
    raise SystemExit(0 if ok else 1)
