#!/bin/bash
# Send Navigation Goal for G1 Robot
# Usage: ./send_goal.sh <x> <y> <yaw_degrees>
# Example: ./send_goal.sh 3 3 0
# export ROS_DOMAIN_ID=1
X=${1:-0.0}
Y=${2:-0.0}
YAW_DEG=${3:-0.0}

# Convert yaw from degrees to quaternion
YAW_RAD=$(echo "scale=5; $YAW_DEG * 3.14159265359 / 180.0" | bc)
QZ=$(echo "scale=5; s($YAW_RAD / 2.0)" | bc -l)
QW=$(echo "scale=5; c($YAW_RAD / 2.0)" | bc -l)

# source /opt/ros/foxy/setup.bash

echo "🎯 Sending navigation goal: ($X, $Y) facing $YAW_DEG°"
echo "   Quaternion: qz=$QZ, qw=$QW"

ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped "
header:
  frame_id: 'map'
pose:
  position:
    x: $X
    y: $Y
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: $QZ
    w: $QW
"

echo "✅ Goal sent! Robot should start moving..."
echo "   Monitor progress with: ros2 topic echo /amcl_pose"



