#!/usr/bin/env python3
"""Read and archive Micolink serial bytes; no ROS/AP output or sensor writes.

Linux TTY input is configured for 115200 8N1. The port is opened read-only;
termios changes host line settings. No command bytes or explicit modem-control
operations are sent; adapter/OS open-close behavior is not established here.
Receipt times are host monotonic observations, never exposure timestamps.
Use synthetic_pty only when the source really is a synthetic test fixture.
"""
from __future__ import annotations

import argparse
import errno
import hashlib
import json
import math
import os
from pathlib import Path
import select
import termios
import time

from xq_autonomy.micolink import MicolinkChronologyGuard, MicolinkParser


def _json(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False)


def _raw_tty(fd):
    attrs = termios.tcgetattr(fd)
    attrs[0] = attrs[1] = attrs[3] = 0
    attrs[2] &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB | getattr(termios, "CRTSCTS", 0))
    attrs[2] |= termios.CS8 | termios.CLOCAL | termios.CREAD
    attrs[4] = attrs[5] = termios.B115200
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


def capture(device, output, duration_s, input_origin):
    if not math.isfinite(duration_s) or not 0 < duration_s <= 3600:
        raise ValueError("duration_s must be finite in (0,3600]")
    if input_origin not in {"synthetic_pty", "hardware_unverified"}:
        raise ValueError("explicit input origin required")
    if output.exists():
        raise ValueError("output must be new; previous captures are preserved")
    parser = MicolinkParser()
    guard = MicolinkChronologyGuard()
    guard.new_session(output.name)
    output.mkdir(parents=True)
    fd = os.open(device, os.O_RDONLY | os.O_NONBLOCK | os.O_NOCTTY)
    chunks = frames = accepted = raw_count = 0
    read_error = None
    started = time.monotonic()
    try:
        if not os.isatty(fd):
            raise ValueError("device must be a TTY; binary replay uses archived chunks")
        _raw_tty(fd)
        ready = {"device": str(device), "input_origin": input_origin,
                 "session_id": output.name, "monotonic_s": time.monotonic(),
                 "sensor_command_bytes_sent": 0, "baud": 115200}
        (output / "capture-ready.json").write_text(_json(ready) + "\n")
        with (output / "raw.bin").open("wb") as raw, (output / "chunks.jsonl").open("w") as chunk_log, (output / "frames.jsonl").open("w") as frame_log:
            while time.monotonic() - started < duration_s:
                remaining = duration_s - (time.monotonic() - started)
                if not select.select([fd], [], [], max(0, min(0.05, remaining)))[0]:
                    continue
                try:
                    data = os.read(fd, 4096)
                except BlockingIOError:
                    continue
                except OSError as exc:
                    if exc.errno != errno.EIO:
                        raise
                    read_error = "tty_io_error_or_peer_closed"
                    break
                receipt = time.monotonic()
                if not data:
                    read_error = "tty_eof"
                    break
                raw.write(data)
                chunk_log.write(_json({"offset": raw_count, "length": len(data), "receipt_monotonic_s": receipt}) + "\n")
                raw_count += len(data)
                chunks += 1
                for message in parser.feed(data):
                    state = guard.observe(message, receipt_monotonic_s=receipt, now_monotonic_s=receipt)
                    frames += 1
                    accepted += int(state["accepted"])
                    frame_log.write(_json({"message": message.to_dict(), "receipt_monotonic_s": receipt,
                                           "chronology": state}) + "\n")
    finally:
        os.close(fd)
    result = {
        "schema_version": 1, "status": "CAPTURED" if read_error is None else "READ_ERROR",
        "input_origin": input_origin, "device": str(device), "duration_s": time.monotonic() - started,
        "bytes": raw_count, "chunks": chunks, "decoded_frames": frames,
        "chronology_accepted_frames": accepted, "parser_counters": dict(parser.counters),
        "buffered_bytes_at_end": parser.buffered_bytes, "read_error": read_error,
        "final_transport_status": guard.status(now_monotonic_s=time.monotonic()),
        "hardware_readiness": "NOT_READY", "acquisition_valid": False,
        "metric_velocity_valid": False, "ap_output_messages": 0,
        "scope": "Raw serial capture and receipt chronology only; no hardware calibration or state fusion",
    }
    result["files"] = {
        name: {"sha256": hashlib.sha256((output / name).read_bytes()).hexdigest(),
               "size_bytes": (output / name).stat().st_size}
        for name in ("raw.bin", "chunks.jsonl", "frames.jsonl")
    }
    (output / "summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    return result


def replay(capture_dir):
    """Recompute from byte offsets and recorded RECEIPT times, without sleeping."""
    raw = (capture_dir / "raw.bin").read_bytes()
    manifest = json.loads((capture_dir / "summary.json").read_text())
    if set(manifest["files"]) != {"raw.bin", "chunks.jsonl", "frames.jsonl"}:
        raise ValueError("unexpected capture file manifest")
    for name, expected in manifest["files"].items():
        data = (capture_dir / name).read_bytes()
        if len(data) != expected["size_bytes"] or hashlib.sha256(data).hexdigest() != expected["sha256"]:
            raise ValueError("capture hash mismatch: " + name)
    guard, parser, frames, offset = MicolinkChronologyGuard(), MicolinkParser(), [], 0
    guard.new_session(capture_dir.name)
    for line in (capture_dir / "chunks.jsonl").read_text().splitlines():
        chunk = json.loads(line)
        if chunk["offset"] != offset or type(chunk["length"]) is not int or chunk["length"] <= 0 or offset + chunk["length"] > len(raw):
            raise ValueError("incomplete or overlapping chunk index")
        end = offset + chunk["length"]
        for message in parser.feed(raw[offset:end]):
            receipt = chunk["receipt_monotonic_s"]
            state = guard.observe(message, receipt_monotonic_s=receipt, now_monotonic_s=receipt)
            frames.append({"message": message.to_dict(), "receipt_monotonic_s": receipt, "chronology": state})
        offset = end
    if offset != len(raw):
        raise ValueError("chunk index does not cover all raw bytes")
    expected = [json.loads(line) for line in (capture_dir / "frames.jsonl").read_text().splitlines()]
    matches = frames == expected and dict(parser.counters) == manifest["parser_counters"] and parser.buffered_bytes == manifest["buffered_bytes_at_end"]
    return {"status": "PASS" if matches else "FAIL", "frames": len(frames), "all_raw_bytes_replayed": offset,
            "input_origin": manifest["input_origin"], "scope": "Deterministic byte/receipt replay; not acquisition-clock validation"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--device", type=Path)
    mode.add_argument("--replay", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--duration-s", type=float, default=10.0)
    parser.add_argument("--input-origin", choices=("synthetic_pty", "hardware_unverified"))
    args = parser.parse_args()
    if args.replay:
        result = replay(args.replay)
    else:
        if args.output is None or args.input_origin is None:
            parser.error("capture requires --output and --input-origin")
        result = capture(args.device, args.output, args.duration_s, args.input_origin)
    print(_json(result))
    return 0 if result["status"] in {"CAPTURED", "PASS"} else 1


if __name__ == "__main__":
    raise SystemExit(main())
