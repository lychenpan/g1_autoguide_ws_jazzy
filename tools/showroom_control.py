#!/usr/bin/env python3
"""
Showroom remote control API (HTTP).

Other modules:
    from showroom_control import next_page, play_video

CLI:
    python showroom_control.py next [--n 2]
    python showroom_control.py playvideo
"""

from __future__ import annotations

import json
import os
import sys
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Any, Optional
from urllib.parse import urljoin

# --- CONFIG (edit here or set SHOWROOM_BASE_URL) ---
DEFAULT_BASE_URL = "http://16a1cd43.r11.cpolar.top"
DEFAULT_TIMEOUT = 30.0
# --- end CONFIG ---


def get_base_url() -> str:
    return os.environ.get("SHOWROOM_BASE_URL", DEFAULT_BASE_URL).rstrip("/")


def get_timeout() -> float:
    raw = os.environ.get("SHOWROOM_HTTP_TIMEOUT")
    if raw is None:
        return DEFAULT_TIMEOUT
    return float(raw)


def join_url(base: str, path: str) -> str:
    if path.startswith("http://") or path.startswith("https://"):
        return path
    return urljoin(base.rstrip("/") + "/", path.lstrip("/"))


@dataclass
class ApiResult:
    status_code: int
    body: str
    data: Optional[dict[str, Any]] = None

    @property
    def ok(self) -> bool:
        return 200 <= self.status_code < 300


def http_get(
    url: str,
    timeout: Optional[float] = None,
) -> ApiResult:
    req = urllib.request.Request(url, method="GET")
    with urllib.request.urlopen(req, timeout=timeout or get_timeout()) as resp:
        status_code = getattr(resp, "status", 200)
        body = resp.read().decode("utf-8", errors="replace").strip()
    data = None
    if body:
        try:
            parsed = json.loads(body)
            if isinstance(parsed, dict):
                data = parsed
        except json.JSONDecodeError:
            pass
    return ApiResult(status_code=status_code, body=body, data=data)


def next_page(
    n: int = 1,
    *,
    base_url: Optional[str] = None,
    timeout: Optional[float] = None,
) -> ApiResult:
    """
    Advance showroom slides (agent next).

    Args:
        n: Number of pages to advance (from 1--5 , corresopond to 5 ppts' PC screen).

    Returns:
        ApiResult with parsed JSON in ``data`` when the server returns JSON.
    """
    base = (base_url or get_base_url()).rstrip("/")
    url = join_url(base, f"/api/agent/next?n={int(n)}")
    return http_get(url, timeout=timeout)


def play_video(
    *,
    base_url: Optional[str] = None,
    timeout: Optional[float] = None,
    command: str = "PlayVideo",
) -> ApiResult:
    """
    Send UDP PlayVideo (or other) command via showroom HTTP API.

    Args:
        command: UDP command name (default ``PlayVideo``).
    """
    base = (base_url or get_base_url()).rstrip("/")
    url = join_url(base, f"/api/send_udp?command={command}")
    return http_get(url, timeout=timeout)



def _print_result(label: str, result: ApiResult) -> None:
    print(f"{label}: HTTP {result.status_code}")
    print(result.body)
    if result.data is not None:
        print(json.dumps(result.data, ensure_ascii=False, indent=2))


def main(argv: Optional[list[str]] = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or argv[0] in ("-h", "--help"):
        print(__doc__.strip())
        print("\nCommands: next [n], playvideo")
        return 0

    cmd = argv[0].lower()
    try:
        if cmd == "next":
            n = int(argv[1]) if len(argv) > 1 else 2
            result = next_page(n=n)
            _print_result("next_page", result)
        elif cmd in ("playvideo", "play_video", "video"):
            result = play_video()
            _print_result("play_video", result)
        else:
            print(f"Unknown command: {cmd}", file=sys.stderr)
            return 2
    except urllib.error.HTTPError as e:
        err = e.read().decode("utf-8", errors="replace")
        print(f"HTTP {e.code}: {err[:500]}", file=sys.stderr)
        return 1
    except urllib.error.URLError as e:
        print(f"Connection error: {e.reason}", file=sys.stderr)
        return 1

    return 0 if result.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
