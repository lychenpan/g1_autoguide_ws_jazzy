#!/bin/bash
# Start TTS web server, run SLAM relocation once, start wireless controller
# listener, then keep TTS and controller running.
set -euo pipefail

TOOLS_DIR="/home/unitree/workspace/dockerws/g1_ws_jazzy/tools"
NODES_DIR="/home/unitree/workspace/dockerws/g1_ws_jazzy/nodes"
PYTHON=/usr/bin/python3
TTS_HOST="0.0.0.0"
TTS_PORT="10011"
TTS_INTERFACE="eth0"
BOOT_DELAY_SEC="${G1_BOOT_DELAY_SEC:-10}"
HEALTH_URL="http://localhost:${TTS_PORT}/health"

cd "$TOOLS_DIR"

if [[ "$BOOT_DELAY_SEC" -gt 0 ]]; then
  sleep "$BOOT_DELAY_SEC"
fi

CONTROLLER_PID=""

"$PYTHON" "$TOOLS_DIR/tts_web_server.py" \
  --host "$TTS_HOST" --port "$TTS_PORT" --interface "$TTS_INTERFACE" &
TTS_PID=$!

cleanup() {
  if [[ -n "$CONTROLLER_PID" ]]; then
    kill "$CONTROLLER_PID" 2>/dev/null || true
  fi
  kill "$TTS_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

for _ in $(seq 1 30); do
  if curl -sf "$HEALTH_URL" >/dev/null; then
    break
  fi
  if ! kill -0 "$TTS_PID" 2>/dev/null; then
    echo "TTS server exited before becoming ready" >&2
    exit 1
  fi
  sleep 2
done

if ! curl -sf "$HEALTH_URL" >/dev/null; then
  echo "TTS server not ready after waiting" >&2
  exit 1
fi

if ! "$PYTHON" "$TOOLS_DIR/g1_slam_relocation.py"; then
  echo "WARNING: SLAM relocation failed; TTS server will keep running" >&2
fi

export PYTHONPATH="${PYTHONPATH:-/home/unitree/workspace/unitree_sdk2_python}:${NODES_DIR}"
"$PYTHON" "$NODES_DIR/unitree_controller_node.py" &
CONTROLLER_PID=$!

if ! kill -0 "$CONTROLLER_PID" 2>/dev/null; then
  echo "Wireless controller node failed to start" >&2
  exit 1
fi

echo "Wireless controller node started (pid=${CONTROLLER_PID})"

wait "$TTS_PID"
