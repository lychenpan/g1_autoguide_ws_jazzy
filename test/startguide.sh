#!/usr/bin/env bash
# Trigger showroom workflow via std_msgs/String on /showroom_mission/start
set -euo pipefail

MESSAGE="${1:-start}"
TOPIC="${SHOWROOM_MISSION_START_TOPIC:-/showroom_mission/start}"

exec python3 "$(dirname "$0")/startguide.py" "$MESSAGE" --topic "$TOPIC"
