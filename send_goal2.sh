#!/bin/bash
# Send Navigation Goal for G1 Robot
# Usage: ./send_goal2.sh <x> <y> <yaw_radians>
# Example: ./send_goal2.sh 3 3 1.57
# export ROS_DOMAIN_ID=1
X=${1:-0.0}
Y=${2:-0.0}
YAW_RAD=${3:-0.0}

# Convert yaw (radians) to quaternion
QZ=$(echo "scale=5; s($YAW_RAD / 2.0)" | bc -l)
QW=$(echo "scale=5; c($YAW_RAD / 2.0)" | bc -l)

# source /opt/ros/foxy/setup.bash

echo "🎯 Sending navigation goal: ($X, $Y) yaw=${YAW_RAD} rad"
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



