#!/usr/bin/env python3
"""
HTTP server that accepts text and speaks it via G1 AudioClient (TTS).

Based on ttstest.py / asr_voice_controller.py patterns.
"""

import argparse
import sys
import threading
import time

sys.path.append("/home/unitree/workspace/unitree_sdk2_python/")

from flask import Flask, jsonify, request
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient

CHARS_PER_SEC = 4.5

app = Flask(__name__)
audio_client = None
speak_lock = threading.Lock()


def init_audio_client(network_interface: str = "eth0", volume: int = 300) -> None:
    global audio_client
    ChannelFactoryInitialize(0, network_interface)
    audio_client = AudioClient()
    audio_client.SetTimeout(10.0)
    audio_client.Init()
    audio_client.SetVolume(volume)


def speak_text(text: str) -> None:
    with speak_lock:
        audio_client.TtsMaker(text, 1)
        time.sleep(len(text) / CHARS_PER_SEC / 4 + 0.5)


@app.route("/health", methods=["GET"])
def health():
    return jsonify({"status": "ok"})


@app.route("/api/speak", methods=["POST"])
def speak():
    data = request.get_json(silent=True) or {}
    text = (data.get("text") or "").strip()
    if not text:
        return jsonify({"error": "missing or empty 'text' field"}), 400

    wait = bool(data.get("wait", False))
    estimated = round(len(text) / CHARS_PER_SEC, 2)

    if wait:
        try:
            speak_text(text)
        except Exception as exc:
            return jsonify({"error": str(exc)}), 500
        return jsonify({"status": "spoken", "text": text, "estimated_duration_sec": estimated})

    threading.Thread(target=speak_text, args=(text,), daemon=True).start()
    return jsonify({"status": "queued", "text": text, "estimated_duration_sec": estimated})


def main():
    parser = argparse.ArgumentParser(description="G1 TTS web server")
    parser.add_argument("--host", default="0.0.0.0", help="Listen address (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=10011, help="Listen port (default: 10011)")
    parser.add_argument("--interface", default="eth0", help="Network interface for DDS (default: eth0)")
    parser.add_argument("--volume", type=int, default=300, help="Speaker volume (default: 300)")
    args = parser.parse_args()

    init_audio_client(args.interface, args.volume)
    print(f"TTS server listening on http://{args.host}:{args.port}")
    print("POST /api/speak  JSON body: {\"text\": \"hello\"}")
    app.run(host=args.host, port=args.port, threaded=True)


if __name__ == "__main__":
    main()
