#!/bin/bash
# Start TTS web server, run SLAM relocation once, start Nav2 in Docker when
# relocation succeeds, start wireless controller listener, then keep running.
set -euo pipefail

TOOLS_DIR="/home/unitree/workspace/dockerws/g1_ws_jazzy/tools"
NODES_DIR="/home/unitree/workspace/dockerws/g1_ws_jazzy/nodes"
PYTHON=(/usr/bin/python3 -u)
TTS_HOST="0.0.0.0"
TTS_PORT="10011"
TTS_INTERFACE="eth0"
BOOT_DELAY_SEC="${G1_BOOT_DELAY_SEC:-10}"
HEALTH_URL="http://localhost:${TTS_PORT}/health"
ROS_SETUP="${ROS_SETUP:-/opt/ros/foxy/setup.bash}"
CYCLONEDDS_SETUP="${CYCLONEDDS_SETUP:-/home/unitree/cyclonedds_ws/install/setup.bash}"
DOCKER_BIN="${DOCKER_BIN:-docker}"
DOCKER_CONTAINER="${G1_DOCKER_CONTAINER:-cp2-jazzy}"
DOCKER_WS_DIR="${G1_DOCKER_WS_DIR:-/workspace/g1_ws_jazzy}"
DOCKER_BASHRC="${G1_DOCKER_BASHRC:-/root/.bashrc}"
HANDSHAKE_DIR="${G1_HANDSHAKE_DIR:-/home/unitree/workspace/teleoperate/handshake}"
HANDSHAKE_PYTHON="${G1_HANDSHAKE_PYTHON:-/home/unitree/anaconda3/envs/tv/bin/python}"
HANDSHAKE_PORT="${G1_HANDSHAKE_PORT:-5000}"
TTS_SPEAK_URL="http://localhost:${TTS_PORT}/api/speak"
G1_CONTROLLER_SPEAK_TEXT="${G1_CONTROLLER_SPEAK_TEXT:-Wireless controller started}"
G1_READY_SPEAK_TEXT="${G1_READY_SPEAK_TEXT:-G1 tools ready, controller and handshake started}"
NAV2_ROS_DOMAIN_ID="${G1_NAV2_ROS_DOMAIN_ID:-1}"
NAV2_VERIFY_TIMEOUT_SEC="${G1_NAV2_VERIFY_TIMEOUT_SEC:-180}"
NAV2_VERIFY_INTERVAL_SEC="${G1_NAV2_VERIFY_INTERVAL_SEC:-5}"
NAV2_REQUIRED_TOPICS="${G1_NAV2_REQUIRED_TOPICS:-/unitree/odom /utlidar/pcl2 /map /cmd_vel /lifecycle_manager_navigation/managed_nodes_activated}"
NAV2_READY_SPEAK_TEXT="${G1_NAV2_READY_SPEAK_TEXT:-Navigation 2 stack ready}"
NAV2_FAIL_SPEAK_TEXT="${G1_NAV2_FAIL_SPEAK_TEXT:-Navigation 2 stack failed to start}"

source_ros_env() {
  if [[ ! -f "$ROS_SETUP" ]]; then
    echo "ROS setup not found: $ROS_SETUP" >&2
    return 1
  fi
  # ROS setup scripts reference unset vars; relax nounset while sourcing.
  set +u
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
  if [[ -f "$CYCLONEDDS_SETUP" ]]; then
    # shellcheck disable=SC1090
    source "$CYCLONEDDS_SETUP"
  fi
  set -u
}

speak_tts() {
  local text="$1"
  local payload

  payload="$("${PYTHON[@]}" -c 'import json,sys; print(json.dumps({"text": sys.argv[1], "wait": True}))' "$text")"
  if ! curl -sf -X POST "$TTS_SPEAK_URL" \
    -H "Content-Type: application/json" \
    -d "$payload"; then
    echo "WARNING: TTS speak request failed" >&2
    return 1
  fi
  echo "TTS spoken: $text"
  return 0
}

start_docker_nav2() {
  local container="$DOCKER_CONTAINER"
  local ws_dir="$DOCKER_WS_DIR"
  local bashrc="$DOCKER_BASHRC"

  if ! command -v "$DOCKER_BIN" >/dev/null 2>&1; then
    echo "docker command not found: $DOCKER_BIN" >&2
    return 1
  fi

  if ! "$DOCKER_BIN" inspect "$container" >/dev/null 2>&1; then
    echo "Docker container not found: $container" >&2
    return 1
  fi

  echo "Starting Docker container: $container"
  if ! "$DOCKER_BIN" restart "$container" >/dev/null; then
    echo "Failed to start Docker container: $container" >&2
    return 1
  fi

  local running=""
  for _ in $(seq 1 30); do
    running="$("$DOCKER_BIN" inspect -f '{{.State.Running}}' "$container" 2>/dev/null || true)"
    if [[ "$running" == "true" ]]; then
      break
    fi
    sleep 1
  done

  if [[ "$running" != "true" ]]; then
    echo "Docker container did not reach running state: $container" >&2
    return 1
  fi

  echo "Launching Nav2 in $container: ${ws_dir}/startnav2.sh (env from ${bashrc})"
  # .bashrc returns early when PS1 is unset (non-interactive). Force full init so
  # nav2_ws / g1_fun_ws setup.bash lines run and cmd_vel_bridge is on PATH.
  if ! "$DOCKER_BIN" exec -d "$container" bash -lc \
    "PS1=1 source \"${bashrc}\" && cd \"${ws_dir}\" && bash startnav2.sh"; then
    echo "Failed to exec startnav2.sh in $container" >&2
    return 1
  fi

  echo "Nav2 launch started inside Docker container: $container"
  return 0
}

docker_nav2_topic_list() {
  "$DOCKER_BIN" exec "$DOCKER_CONTAINER" bash -lc \
    "PS1=1 source \"${DOCKER_BASHRC}\" && export ROS_DOMAIN_ID=${NAV2_ROS_DOMAIN_ID} && ros2 topic list" \
    2>/dev/null || true
}

verify_docker_nav2() {
  local deadline=$((SECONDS + NAV2_VERIFY_TIMEOUT_SEC))
  local topics_list missing

  echo "Waiting for Nav2 on ROS domain ${NAV2_ROS_DOMAIN_ID} (timeout ${NAV2_VERIFY_TIMEOUT_SEC}s)..."
  echo "Required topics:${NAV2_REQUIRED_TOPICS}"

  while (( SECONDS < deadline )); do
    topics_list="$(docker_nav2_topic_list)"
    if [[ -n "$topics_list" ]]; then
      missing=""
      for topic in $NAV2_REQUIRED_TOPICS; do
        if ! grep -Fxq "$topic" <<< "$topics_list"; then
          missing="${missing} ${topic}"
        fi
      done
      if [[ -z "$missing" ]]; then
        echo "Nav2 verification OK; all required topics present"
        speak_tts "$NAV2_READY_SPEAK_TEXT" || true
        return 0
      fi
      echo "Nav2 topics still missing:${missing}"
    else
      echo "Nav2 topic list empty or unavailable"
    fi
    sleep "$NAV2_VERIFY_INTERVAL_SEC"
  done

  echo "WARNING: Nav2 verification timed out after ${NAV2_VERIFY_TIMEOUT_SEC}s" >&2
  speak_tts "$NAV2_FAIL_SPEAK_TEXT" || true
  return 1
}

cd "$TOOLS_DIR"

if [[ "$BOOT_DELAY_SEC" -gt 0 ]]; then
  sleep "$BOOT_DELAY_SEC"
fi

CONTROLLER_PID=""
HANDSHAKE_PID=""
NAV2_LAUNCHED=0

"${PYTHON[@]}" "$TOOLS_DIR/tts_web_server.py" \
  --host "$TTS_HOST" --port "$TTS_PORT" --interface "$TTS_INTERFACE" &
TTS_PID=$!

cleanup() {
  if [[ -n "$HANDSHAKE_PID" ]]; then
    kill "$HANDSHAKE_PID" 2>/dev/null || true
  fi
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

if ! source_ros_env; then
  echo "SLAM relocation odom check requires ROS; setup failed" >&2
  exit 1
fi

if "${PYTHON[@]}" "$TOOLS_DIR/g1_slam_relocation.py"; then
  echo "SLAM relocation succeeded; starting Docker Nav2 stack..."
  if start_docker_nav2; then
    NAV2_LAUNCHED=1
  else
    echo "WARNING: Docker Nav2 startup failed; TTS server and controller will still start" >&2
  fi
else
  echo "WARNING: SLAM relocation failed; skipping Docker Nav2 startup" >&2
fi

export PYTHONPATH="/home/unitree/workspace/unitree_sdk2_python:${PYTHONPATH}:${NODES_DIR}"

if ! "${PYTHON[@]}" -c "import rclpy; from std_msgs.msg import String"; then
  echo "Wireless controller node requires rclpy; import check failed" >&2
  exit 1
fi

"${PYTHON[@]}" "$NODES_DIR/unitree_controller_node.py" &
CONTROLLER_PID=$!

for _ in $(seq 1 20); do
  if ! kill -0 "$CONTROLLER_PID" 2>/dev/null; then
    status=0
    wait "$CONTROLLER_PID" 2>/dev/null || status=$?
    echo "Wireless controller node exited during startup (exit=${status})" >&2
    exit 1
  fi
  sleep 0.25
done

echo "Wireless controller node started (pid=${CONTROLLER_PID})"
speak_tts "$G1_CONTROLLER_SPEAK_TEXT" || true

handshake_ready=0
if [[ ! -x "$HANDSHAKE_PYTHON" ]]; then
  echo "WARNING: Handshake python not found: $HANDSHAKE_PYTHON" >&2
elif [[ ! -f "$HANDSHAKE_DIR/handshake_server.py" ]]; then
  echo "WARNING: Handshake server script not found: $HANDSHAKE_DIR/handshake_server.py" >&2
else
  echo "Starting handshake server in $HANDSHAKE_DIR (conda env tv)..."
  (
    cd "$HANDSHAKE_DIR"
    exec "$HANDSHAKE_PYTHON" -u "$HANDSHAKE_DIR/handshake_server.py"
  ) &
  HANDSHAKE_PID=$!

  handshake_ready=0
  for _ in $(seq 1 60); do
    if curl -sf "http://localhost:${HANDSHAKE_PORT}/status" >/dev/null; then
      handshake_ready=1
      break
    fi
    if ! kill -0 "$HANDSHAKE_PID" 2>/dev/null; then
      echo "WARNING: Handshake server exited during startup" >&2
      HANDSHAKE_PID=""
      break
    fi
    sleep 1
  done

  if [[ "$handshake_ready" -eq 1 ]]; then
    echo "Handshake server ready on port ${HANDSHAKE_PORT} (pid=${HANDSHAKE_PID})"
  elif [[ -n "$HANDSHAKE_PID" ]]; then
    echo "WARNING: Handshake server not responding on /status after 60s (pid=${HANDSHAKE_PID})" >&2
  fi
fi

if [[ "$handshake_ready" -eq 1 ]] && kill -0 "$CONTROLLER_PID" 2>/dev/null; then
  echo "Controller and handshake ready; sending startup hint to TTS..."
  speak_tts "handshake server started" || true
fi

if [[ "$NAV2_LAUNCHED" -eq 1 ]]; then
  verify_docker_nav2 || true
fi

wait "$TTS_PID"


