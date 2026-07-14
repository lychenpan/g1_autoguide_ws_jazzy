#!/usr/bin/env python3
"""Convert ROS log epoch timestamps to human-readable form (on-demand).

Not run automatically at Nav2 startup. After startnav2.sh, the previous raw
run is kept at ~/.ros/log_last by ros_log_cleanup.sh. Convert when debugging:

  python3 tools/log_convert.py --ros-log-dir ~/.ros/log_last

By default reads ~/.ros/log/ and writes to ~/.ros/log_readable/ (replacing any
existing log_readable contents).
"""
from __future__ import annotations

import argparse
import re
import shutil
import sys
from datetime import datetime
from pathlib import Path

EPOCH_IN_BRACKETS = re.compile(r"\[(\d+\.\d+)\]")
EPOCH_AT_LINE_START = re.compile(r"^(\d+\.\d+)\s")


def _format_epoch(epoch_text: str) -> str:
    ts = float(epoch_text)
    dt = datetime.fromtimestamp(ts)
    micros = int(round((ts % 1) * 1_000_000))
    return f"{dt.strftime('%Y-%m-%d %H:%M:%S')}.{micros:06d}"


def convert_line(line: str) -> str:
    line = EPOCH_IN_BRACKETS.sub(lambda m: f"[{_format_epoch(m.group(1))}]", line)
    return EPOCH_AT_LINE_START.sub(lambda m: f"{_format_epoch(m.group(1))} ", line)


def convert_file(src: Path, dst: Path) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    with src.open(encoding="utf-8", errors="replace") as fin, dst.open(
        "w", encoding="utf-8"
    ) as fout:
        for line in fin:
            fout.write(convert_line(line))


def find_latest_session(log_dir: Path) -> Path | None:
    sessions = [
        path
        for path in log_dir.iterdir()
        if path.is_dir() and not path.is_symlink() and path.name[:4].isdigit()
    ]
    if not sessions:
        return None
    return max(sessions, key=lambda path: path.stat().st_mtime)


def convert_tree(src_dir: Path, dst_dir: Path) -> int:
    count = 0
    for src in sorted(src_dir.rglob("*.log")):
        rel = src.relative_to(src_dir)
        convert_file(src, dst_dir / rel)
        count += 1
    return count


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--ros-log-dir",
        default=None,
        help="ROS log root (default: $ROS_LOG_DIR or ~/.ros/log)",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="Readable output root (default: <ros-log-dir>/../log_readable)",
    )
    args = parser.parse_args(argv)

    ros_log_dir = Path(
        args.ros_log_dir or __import__("os").environ.get("ROS_LOG_DIR", "~/.ros/log")
    ).expanduser()
    output_dir = Path(
        args.output_dir
        if args.output_dir
        else ros_log_dir.parent / "log_readable"
    ).expanduser()

    if not ros_log_dir.is_dir():
        print(f"log_convert: no ROS log dir at {ros_log_dir}, skipping", file=sys.stderr)
        return 0

    if output_dir.exists():
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True)

    converted = 0
    latest_session = find_latest_session(ros_log_dir)
    if latest_session is not None:
        session_out = output_dir / latest_session.name
        converted += convert_tree(latest_session, session_out)
        (output_dir / "_latest_session.txt").write_text(
            f"{latest_session}\n", encoding="utf-8"
        )

    flat_out = output_dir / "_flat"
    flat_logs = sorted(ros_log_dir.glob("*.log"), key=lambda p: p.stat().st_mtime)
    for src in flat_logs:
        convert_file(src, flat_out / src.name)
        converted += 1

    print(
        f"log_convert: wrote {converted} file(s) to {output_dir}"
        + (f" (latest session: {latest_session.name})" if latest_session else ""),
        file=sys.stderr,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
