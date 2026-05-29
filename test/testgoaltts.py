import math
import os
import sys
import time

_TOOLS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools')
if _TOOLS_DIR not in sys.path:
    sys.path.insert(0, os.path.normpath(_TOOLS_DIR))

# Unitree SDK DDS (domain 0) must initialize before rclpy import/init.
_SDK_DIR = os.environ.get(
    'UNITREE_SDK2PY_PATH',
    '/home/unitree/workspace/unitree_sdk2_python',
)
if _SDK_DIR not in sys.path:
    sys.path.append(_SDK_DIR)

from unitree_sdk2py.core.channel import ChannelFactoryInitialize  # noqa: E402

ChannelFactoryInitialize(0, os.environ.get('UNITREE_NET_IFACE', 'eth0'))
os.environ.setdefault('ROS_DOMAIN_ID', '1')

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from tts_player import RemoteTTSPlayer   # TTS 已禁用


GOALS = [
    # (5.362529, 6.215906, -1.935613), #展厅定点测试点
    # (5.362529, 6.215906, -1.935613),
    # (5.362529, 6.215906, -1.935613),
    # (5.362529, 6.215906, -1.935613),
    # (5.362529, 6.215906, -1.935613),
    # (-4.807027, 17.611385, 1.231883), #吊架起吊点
    # (-4.807027, 17.611385, 1.231883),
    # (-4.807027, 17.611385, 1.231883),
    # (-4.807027, 17.611385, 1.231883),
    # (-4.807027, 17.611385, 1.231883),
    # (-4.807027, 17.611385, 1.231883),

    (-6.770853, 14.360708, 1.421385), #会客厅内两点
    (-6.664255, 15.790245, 3.862700),
    (-6.859162, 18.303812, -2.745892),
    (-5.589580, 16.523210, -0.542670),

    # (-6.820897, 15.658363, -0.817101),
    # (-6.820897, 15.658363, 0.317101),
    # (-6.820897, 15.658363, 1.817101),
    # (-6.820897, 15.658363, 3.1417101),
    # (-6.820897, 15.658363, 4.537101),


    # (-6.122174, 16.459642, -3.004197),
    # (-7.260242, 16.190132, -2.990012),
  
    # (-4.810816, 9.176082, -2.387978),  #电梯口迎宾点
    # (-0.067669, 7.151849, -0.696831),
    # (1.431891, 5.442519, 0.895148),    #展厅玻璃门口两点
    # (2.313706, 6.542465, 0.751393),         
    # (9.724871, 7.990585, -3.111007),   #展厅内讲解点
    # (3.640587, 9.509808, -1.498545),
    # (5.552542, 11.950672, -1.851832),
    # (8.355801, 9.960559, -2.709656),
    # (5.827041, -1.859886, 2.292660),
    # (4.296857, -4.625527, 1.332774),
    # (-0.743927, -3.763364, 0.346843),
    # (5.65731, 17.61633, -2.41657),    #展厅最后两个屏幕定位点 未测量
    # (-3.55489, 17.98313, -0.36193),  
]


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


def split_text_into_parts(text: str, parts: int):
    # 此函数已不再使用，但保留以免影响其他代码
    text = text.strip()
    if parts <= 0:
        return []
    if not text:
        return [""] * parts

    length = len(text)
    chunks = []
    for i in range(parts):
        start = round(i * length / parts)
        end = round((i + 1) * length / parts)
        chunks.append(text[start:end].strip())
    return chunks


class NavClient(Node):
    def __init__(self):
        super().__init__("nav_client")
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def navigate_blocking(self, x: float, y: float, yaw: float, timeout_sec: float = 300.0):
        # 等待 action server 可用
        self.get_logger().info("Step[NAV]: waiting for /navigate_to_pose action server")
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("navigate_to_pose server not available")
            return None

        # 构造 goal
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

        # 发送并阻塞等待是否接受
        self.get_logger().info(
            f"Step[NAV]: sending goal x={x:.5f}, y={y:.5f}, yaw={yaw:.5f}, qz={qz:.5f}, qw={qw:.5f}"
        )
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout_sec)
        if not send_future.done():
            self.get_logger().error("timeout waiting goal acceptance")
            return None
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("goal rejected")
            return None
        self.get_logger().info("Step[NAV]: goal accepted by action server")

        # 阻塞等待最终结果
        self.get_logger().info("Step[NAV]: waiting for navigation result")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        if not result_future.done():
            self.get_logger().error("timeout waiting goal result")
            return None

        wrapped = result_future.result()
        self.get_logger().info(
            f"Step[NAV]: result received status={wrapped.status}({nav_status_to_text(wrapped.status)})"
        )
        return wrapped.status, wrapped.result


def main():
    tts = RemoteTTSPlayer()   # TTS 已禁用
    rclpy.init()
    node = NavClient()
    try:
        node.get_logger().info("=== Robot nav mission started (TTS disabled) ===")
        # 以下 TTS 相关代码全部注释
        with open("./1.txt", "r", encoding="utf-8") as f:
            full_text = f.read()
        speech_chunks = split_text_into_parts(full_text, 8)
        node.get_logger().info(
            f"Loaded speech text: total_chars={len(full_text.strip())}, chunks={len(speech_chunks)}"
        )

        for idx, (x, y, yaw) in enumerate(GOALS, start=1):
            node.get_logger().info(f"--- Point {idx}/{len(GOALS)} begin ---")
            node.get_logger().info(f"Step[NAV]: start navigate to point {idx}: x={x}, y={y}, yaw={yaw}")

            out = node.navigate_blocking(x, y, yaw, timeout_sec=300.0)
            node.get_logger().info(f"Step[NAV]: point {idx} raw result: {out}")

            if out is None or out[0] != GoalStatus.STATUS_SUCCEEDED:
                status_text = "None" if out is None else f"{out[0]}({nav_status_to_text(out[0])})"
                node.get_logger().warn(
                    f"Step[NAV]: point {idx} not successful, status={status_text}, continue."
                )
                continue

            node.get_logger().info(f"Step[NAV]: point {idx} navigation succeeded")

            print("test sleep")
            time.sleep(5)
            continue
            # 所有 TTS 播放及等待均被注释
            if idx >= 0:
                chunk_idx = idx - 0
                speak_text = speech_chunks[chunk_idx] if chunk_idx < len(speech_chunks) else ""
                if speak_text:
                    node.get_logger().info("Step[TTS]: wait 1 second before speaking")
                    time.sleep(1.0)
                    node.get_logger().info(f"Step[TTS]: start speaking point {idx}, chunk {chunk_idx + 1}/8, chars={len(speak_text)}")
                    print(speak_text)
                    tts.playtext(speak_text)
                    tts.wait_done()
                    node.get_logger().info(f"Step[TTS]: end speaking at point {idx}")
                    node.get_logger().info("Step[FLOW]: wait 2 seconds before next navigation")
                    time.sleep(2.0)
                else:
                    node.get_logger().warn(f"Step[TTS]: point {idx} has empty chunk, skip speaking.")
            else:
                node.get_logger().info(f"Step[TTS]: point {idx} is before point 3, no speaking required")
            node.get_logger().info(f"--- Point {idx}/{len(GOALS)} end ---")
            # 可选：添加一点延时避免连续导航过于紧密（根据实际需要决定是否保留）
            time.sleep(1.0)

        node.get_logger().info("=== Mission complete: all points processed ===")
    finally:
        node.get_logger().info("Step[SHUTDOWN]: shutting down ROS node")
        tts.stop()   # TTS 已禁用
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
