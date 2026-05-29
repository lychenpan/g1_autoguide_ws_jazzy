#!/usr/bin/env python3
# Suppress verbose MediaPipe/TFLite and matplotlib warnings (set before imports)
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
os.environ['GLOG_minloglevel'] = '3'

import warnings
warnings.filterwarnings('ignore', message='Unable to import Axes3D')
warnings.filterwarnings('ignore', message='multiple versions of Matplotlib')

import json
import time
import urllib.request
import urllib.error

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np

# ============ Global parameters ============
# Handshake server HTTP API (replace ROS2 topic)
# From Docker: use host.docker.internal or host IP. From host: http://localhost:5000
HANDSHAKE_SERVER_URL = os.environ.get("HANDSHAKE_SERVER_URL", "http://localhost:5000")

# Whether to save and visualize related data (RGB, depth, etc.)
SAVEVIS = False
# Whether to log debug info for each detected hand case
LOG_DEBUG = False
# Minimum duration (seconds) of continuous hand detection before printing position info
HAND_DURATION_THRESHOLD = 2.0
# Max number of recent hand detections to keep for history
MAX_RECENT_HANDS = 20

# Output directory for saved frames
OUTPUT_DIR = os.path.join(os.path.dirname(__file__), 'saved_frames')


flag = False


class Hand3DNode(Node):

    def __init__(self):
        super().__init__('hand_3d_node')

        self.bridge = CvBridge()

        # Subscribers
        self.create_subscription(Image,'/camera/camera/color/image_raw',self.rgb_callback,10)
        self.create_subscription(Image,'/camera/camera/aligned_depth_to_color/image_raw',self.depth_callback,10)
        self.create_subscription(CameraInfo,'/camera/camera/color/camera_info',self.info_callback,10)

        # HTTP API for hand position (x;y;z) to handshake_server
        self.handd_url = f"{HANDSHAKE_SERVER_URL.rstrip('/')}/handshake/handd"
        print(f"Handshake API: POST {self.handd_url} (x;y;z)")

        # MediaPipe
        self.mp_hands = mp.solutions.hands
        self.mp_draw = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.5
        )

        self.latest_depth = None
        self.fx = self.fy = self.cx = self.cy = None
        self.saved_once = False

        # Transformation matrix from camera_color_optical_frame to base_link (G1 robot)
        # Same convention as depth_camera_node.py
        self.T_base_optical = np.array([
            [4.13679693e-17, -7.37277337e-01,  6.75590208e-01,  5.76235000e-02],
            [-1.00000000e+00,  3.74939946e-33,  6.12323400e-17,  1.75300000e-02],
            [-4.51452165e-17, -6.75590208e-01, -7.37277337e-01,  1.29870000e+00],
            [0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]
        ])

        # Hand detection duration tracking
        self._hand_first_detected_at = None
        self._hand_last_position = None
        self._hand_duration_printed = False

        # Recent hand detections for history
        self._recent_hands = []

        print("Hand3DNode initialized")

    def info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def _save_and_visualize_data(self, rgb, depth_vis, depth_raw, hand_pos=None, rgb_init=None):
        """Save RGB, depth visualization, raw depth and camera_info when SAVEVIS is True.
        hand_pos: dict with 'u', 'v', 'X', 'Y', 'Z' for drawing circle and putText; None to skip.
        rgb_init: unmodified BGR frame (before landmarks / overlays); saved as rgb_init.png.
        """
        if not SAVEVIS or self.saved_once:
            return
        print("--------------------------------")
        if self.fx is None:
            self.get_logger().warn('Cannot save: camera_info not yet received')
            return
        rgb_to_save = rgb.copy()
        if hand_pos is not None:
            u, v = hand_pos['u'], hand_pos['v']
            X, Y, Z = hand_pos['X'], hand_pos['Y'], hand_pos['Z']
            cv2.circle(rgb_to_save, (u, v), 8, (0, 255, 0), -1)
            text = f"X:{X:.2f} Y:{Y:.2f} Z:{Z:.2f} m"
            cv2.putText(rgb_to_save, text, (u + 10, v),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        # Raw camera RGB with no hand overlays or landmarks
        if rgb_init is not None:
            cv2.imwrite(os.path.join(OUTPUT_DIR, 'rgb_init.png'), rgb_init)
        cv2.imwrite(os.path.join(OUTPUT_DIR, 'rgb.png'), rgb_to_save)
        cv2.imwrite(os.path.join(OUTPUT_DIR, 'depth_vis.png'), depth_vis)
        np.save(os.path.join(OUTPUT_DIR, 'depth_raw.npy'), depth_raw)
        with open(os.path.join(OUTPUT_DIR, 'camera_info.json'), 'w') as f:
            json.dump({'fx': self.fx, 'fy': self.fy, 'cx': self.cx, 'cy': self.cy}, f, indent=2)
        self.get_logger().info(
            f'Saved rgb_init.png (no viz), rgb.png, depth and camera_info to {OUTPUT_DIR}'
        )
        self.saved_once = True

    def _record_hand_and_check_duration(self, pos_3d, timestamp):
        """Record hand detection and print info if it lasts >= HAND_DURATION_THRESHOLD."""
        # Reset tracking when hand was lost
        if self._hand_first_detected_at is None:
            self._hand_first_detected_at = timestamp
            self._hand_duration_printed = False

        self._hand_last_position = pos_3d

        # Append to recent history
        hand_info = {
            'timestamp': timestamp,
            'position': pos_3d,
            'duration': timestamp - self._hand_first_detected_at,
        }
        self._recent_hands.append(hand_info)
        if len(self._recent_hands) > MAX_RECENT_HANDS:
            self._recent_hands.pop(0)

        global flag

        # if flag:
        #     return

        # Check if hand has been detected for >= 2 seconds
        elapsed = timestamp - self._hand_first_detected_at
        if elapsed >= HAND_DURATION_THRESHOLD and not self._hand_duration_printed:
            self._hand_duration_printed = True
            flag = True
            pos = pos_3d
            # POST "x;y;z" to handshake_server HTTP API
            data = f"{pos['X']:.6f};{pos['Y']:.6f};{pos['Z']:.6f}".encode("utf-8")
            # print(f"data: {data}")
            # return
            try:
                req = urllib.request.Request(
                    self.handd_url,
                    data=data,
                    method="POST",
                    headers={"Content-Type": "text/plain"},
                )
                with urllib.request.urlopen(req, timeout=5):
                    print(f"[Hand 2s] HTTP handd request sent OK")
            except urllib.error.HTTPError as e:
                print(f"[Hand 2s] HTTP handd error {e.code}: {e.reason}")
            except urllib.error.URLError as e:
                print(f"[Hand 2s] HTTP handd URL error: {e.reason}")
            except Exception as e:
                print(f"[Hand 2s] HTTP handd failed: {e}")
            print(f"[Hand 2s] Detected hand in vision field for {elapsed:.1f}s")
            print(f"[Hand 2s] Current position: X={pos['X']:.3f} Y={pos['Y']:.3f} Z={pos['Z']:.3f} m")
            print(f"[Hand 2s] Recent detections ({len(self._recent_hands)}):")
        

    def _reset_hand_duration_tracking(self):
        """Reset when hand is no longer detected."""
        self._hand_first_detected_at = None
        self._hand_last_position = None
        self._hand_duration_printed = False

    def _optical_to_base_link(self, x_opt, y_opt, z_opt):
        """Transform point from camera optical frame to base_link frame."""
        p_opt = np.array([x_opt, y_opt, z_opt, 1.0])
        p_base = (self.T_base_optical @ p_opt)[:3]
        return p_base[0], p_base[1], p_base[2]

    def rgb_callback(self, msg):
        if self.latest_depth is None or self.fx is None:
            return

        rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Snapshot before any drawing (landmarks, circles) for rgb_init.png
        rgb_init = rgb.copy()
        rgb_for_mediapipe = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_for_mediapipe)

        pos_3d = None
        hand_pos_to_save = None  # for circle/putText in _save_and_visualize_data

        if results.multi_hand_landmarks:
            hand_landmarks = results.multi_hand_landmarks[0]

            # draw landmarks
            self.mp_draw.draw_landmarks(
                rgb,
                hand_landmarks,
                self.mp_hands.HAND_CONNECTIONS
            )

            # use palm center
            lm = hand_landmarks.landmark[9]
            h, w, _ = rgb.shape
            u = int(lm.x * w)
            v = int(lm.y * h)

            # depth window
            window = 3
            d = self.latest_depth[max(0, v - window):v + window + 1,
                                  max(0, u - window):u + window + 1]
            z = np.median(d) / 1000.0

            if z > 0:
                # 3D point in camera optical frame
                X_opt = (u - self.cx) * z / self.fx
                Y_opt = (v - self.cy) * z / self.fy
                Z_opt = z
                # Transform to base_link frame
                X, Y, Z = self._optical_to_base_link(X_opt, Y_opt, Z_opt)
                pos_3d = {'X': X, 'Y': Y, 'Z': Z}
                hand_pos_to_save = {'u': u, 'v': v, 'X': X, 'Y': Y, 'Z': Z}

            # Hand duration tracking and 2-second logic
            if pos_3d is not None:
                t = time.time()
                self._record_hand_and_check_duration(pos_3d, t)
            else:
                self._reset_hand_duration_tracking()

            # Debug logging per detected case
            if LOG_DEBUG:
                self.get_logger().info(
                    f"[DEBUG] Hand detected: u={u} v={v} pos_3d={pos_3d} "
                    f"landmarks_count={len(hand_landmarks.landmark)}"
                )
        else:
            self._reset_hand_duration_tracking()

        

        # Save and visualize (only when SAVEVIS is True)
        if SAVEVIS:
            # Depth visualization
            depth_vis = cv2.convertScaleAbs(self.latest_depth, alpha=0.03)
            depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
            self._save_and_visualize_data(
                rgb, depth_vis, self.latest_depth, hand_pos_to_save, rgb_init=rgb_init
            )


def main(args=None):

    rclpy.init(args=args)

    node = Hand3DNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()