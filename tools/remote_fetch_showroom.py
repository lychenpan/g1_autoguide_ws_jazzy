#!/usr/bin/env python3
"""
Remote Showroom client: fetch speak text per slide only (no PPT download).

Edit CONFIG below, then run:
  python remote_fetch_showroom.py

Writes showroom_speak.json (speakByPage + slideCount per slot, plus fixedspeaktext).
"""

from __future__ import annotations

import json
import sys
import urllib.error
import urllib.request
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
from urllib.parse import urljoin

SCRIPT_DIR = Path(__file__).resolve().parent

# --- CONFIG (edit here) ---
BASE_URL = "http://112.95.75.67:15000"
HTTP_TIMEOUT = 120.0
OUTPUT_FILE = SCRIPT_DIR / "../data" / "showroom_speak.json"
# --- end CONFIG ---


def join_url(base: str, path: str) -> str:
    if path.startswith("http://") or path.startswith("https://"):
        return path
    return urljoin(base.rstrip("/") + "/", path.lstrip("/"))


def get_json(base_url: str, path: str, timeout: float) -> dict[str, Any]:
    url = join_url(base_url, path)
    req = urllib.request.Request(url, headers={"Accept": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        data = json.loads(resp.read().decode("utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"{path}: expected JSON object")
    return data


def fetch_files_by_slot(base_url: str, timeout: float) -> dict[int, dict[str, Any]]:
    files = get_json(base_url, "/api/files", timeout)
    return {
        int(row["slot"]): row
        for row in files.get("slots") or []
        if "slot" in row
    }


def fetch_slide_count(base_url: str, slot: int, timeout: float) -> int | None:
    try:
        meta = get_json(base_url, f"/api/preview/{slot}", timeout)
    except urllib.error.HTTPError:
        return None
    count = meta.get("slideCount")
    return int(count) if isinstance(count, int) else None


def fetch_fixed_speak_texts(base_url: str, timeout: float) -> list[str]:
    data = get_json(base_url, "/api/fixed-speak", timeout)
    texts = data.get("texts")
    if not isinstance(texts, list):
        raise ValueError("/api/fixed-speak: response missing texts list")
    return [str(t) for t in texts]


def main() -> int:
    OUTPUT_FILE.parent.mkdir(parents=True, exist_ok=True)

    try:
        data = get_json(BASE_URL, "/api/speak/all", HTTP_TIMEOUT)
        files_by_slot = fetch_files_by_slot(BASE_URL, HTTP_TIMEOUT)
        fixed_speak_texts = fetch_fixed_speak_texts(BASE_URL, HTTP_TIMEOUT)
    except urllib.error.HTTPError as e:
        err = e.read().decode("utf-8", errors="replace")
        print(f"HTTP {e.code} {e.reason}: {err[:500]}", file=sys.stderr)
        return 1
    except urllib.error.URLError as e:
        print(f"Connection error: {e.reason}", file=sys.stderr)
        return 1
    except (json.JSONDecodeError, ValueError) as e:
        print(f"Bad response: {e}", file=sys.stderr)
        return 1

    slots_out: list[dict[str, Any]] = []
    for row in data.get("slots") or []:
        if "slot" not in row:
            continue
        slot = int(row["slot"])
        speak = row.get("speakByPage") or {}
        speak = {str(k): str(v) for k, v in speak.items()}

        file_row = files_by_slot.get(slot, {})
        present = bool(file_row.get("present"))
        slide_count = fetch_slide_count(BASE_URL, slot, HTTP_TIMEOUT) if present else None

        slots_out.append(
            {
                "slot": slot,
                "slideCount": slide_count,
                "speakByPage": speak,
            }
        )

    export = {
        "version": data.get("version", 1),
        "fetchedAt": datetime.now(timezone.utc).isoformat(),
        "baseUrl": BASE_URL.rstrip("/"),
        "speakBinding": data.get("speakBinding") or "slot+page",
        "fixedspeaktext": fixed_speak_texts,
        "slots": slots_out,
    }

    OUTPUT_FILE.write_text(
        json.dumps(export, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    print(
        f"Wrote {OUTPUT_FILE} (slots={len(slots_out)}, fixedspeaktext={len(fixed_speak_texts)})",
        file=sys.stderr,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
