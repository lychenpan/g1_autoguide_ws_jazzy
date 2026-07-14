#!/bin/bash
# Preserve the previous Nav2 run, then clear ROS_LOG_DIR for a fresh launch.
#
# Previous run (raw logs) is kept at ROS_LOG_KEEP_DIR (default: /root/.ros/log_last).
# Only one previous run is retained; older archives are replaced.
#
# Convert to readable timestamps only when needed (manual):
#   python3 tools/log_convert.py --ros-log-dir /root/.ros/log_last
set -euo pipefail

ROS_LOG_DIR="${ROS_LOG_DIR:-/root/.ros/log}"
ROS_LOG_KEEP_DIR="${ROS_LOG_KEEP_DIR:-/root/.ros/log_last}"

if [[ -d "$ROS_LOG_DIR" ]] && [[ -n "$(ls -A "$ROS_LOG_DIR" 2>/dev/null)" ]]; then
  rm -rf "$ROS_LOG_KEEP_DIR"
  mv "$ROS_LOG_DIR" "$ROS_LOG_KEEP_DIR"
  echo "ros_log_cleanup: kept previous run at ${ROS_LOG_KEEP_DIR}"
fi

mkdir -p "$ROS_LOG_DIR"
