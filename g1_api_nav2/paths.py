"""Workspace path helpers (no colcon install required)."""
import os

G1_API_DIR = os.path.abspath(os.path.dirname(__file__))
WS_ROOT = os.path.abspath(os.path.join(G1_API_DIR, '..'))

PARAMS_YAML = os.path.join(G1_API_DIR, 'params.yaml')
MAP_YAML = os.path.join(G1_API_DIR, 'map.yaml')
MAP_PNG = os.path.join(G1_API_DIR, 'map.png')
LAUNCH_FILE = os.path.join(G1_API_DIR, 'unitree_slam_bringup.launch.py')
RELOCATION_ODOM_BRIDGE = os.path.join(WS_ROOT, 'nodes', 'unitree_relocation_odom_bridge.py')
