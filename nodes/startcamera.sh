#!/bin/bash
# Minimal RealSense bringup for hand detection (camera_handdet_http.py).
#
# Publishes:
#   /camera/camera/color/image_raw
#   /camera/camera/aligned_depth_to_color/image_raw
#   /camera/camera/color/camera_info
#
# Extra args: ./startcamera.sh pointcloud.enable:=true output:=log

set -euo pipefail

CPU_CORE="${REALSENSE_CPU_CORE:-5}"

exec taskset -c "${CPU_CORE}" ros2 launch realsense2_camera rs_launch.py \
  depth_module.depth_profile:=640,480,15 \
  rgb_camera.color_profile:=640,480,15 \
  align_depth.enable:=true \
  depth_module.emitter_enabled:=0 \
  spatial_filter.enable:=true \
  temporal_filter.enable:=true \
  hole_filling_filter.enable:=true \
  pointcloud.enable:=false \
  publish_tf:=false \
  "$@"
