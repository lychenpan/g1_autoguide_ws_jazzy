#!/usr/bin/env bash
# cmd_vel_bridge is started by unitree_slam_bringup_jazzy.launch.py (phase 2).
# This wrapper is kept for manual/debug use.
set -euo pipefail
exec ros2 run cmd_vel_bridge cmd_vel_bridge --ros-args \
  -p print_cmd_vel_log:=true \
  -p enable_min_yaw_clamp:=false