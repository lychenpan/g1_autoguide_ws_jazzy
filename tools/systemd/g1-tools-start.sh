#!/bin/bash
# Start TTS web server, run SLAM relocation once, then keep TTS running.
set -euo pipefail

TOOLS_DIR="/home/unitree/workspace/dockerws/g1_ws_jazzy/tools"
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

"$PYTHON" "$TOOLS_DIR/tts_web_server.py" \
  --host "$TTS_HOST" --port "$TTS_PORT" --interface "$TTS_INTERFACE" &
TTS_PID=$!

cleanup() {
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

wait "$TTS_PID"
