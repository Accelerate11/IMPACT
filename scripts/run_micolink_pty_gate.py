#!/usr/bin/env python3
"""Exercise the real Linux TTY capture path with explicitly synthetic bytes.

The fixture is produced from the documented frame layout, independently of the
decoder. This is not a sensor capture, firmware interoperability test or flight.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import pty
import struct
import subprocess
import sys
import time


def frame(sequence, time_ms, *, message_id=0x51, invalid_measurement=False):
    payload = struct.pack("<IIBBBBhhBBH", time_ms, 0 if invalid_measurement else 1500,
                          200, 1, 0 if invalid_measurement else 1, 0,
                          239, -125, 0 if invalid_measurement else 220,
                          0 if invalid_measurement else 1, 0)
    content = bytes((0xEF, 0x0F, 0, message_id, sequence, len(payload))) + payload
    return content + bytes((sum(content) & 0xFF,))


def run(output):
    if output.exists():
        raise ValueError("output must be new; failed attempts are preserved")
    output.mkdir(parents=True)
    capture = output / "capture"
    script = Path(__file__).resolve().with_name("capture_micolink.py")
    master, slave = pty.openpty()
    device = os.ttyname(slave)
    corrupt = bytearray(frame(2, 1020))
    corrupt[-1] ^= 0x80
    wire = (b"noise\x00" + bytes.fromhex("ef0f00510041") + frame(0, 1000)
            + frame(1, 1010) + bytes(corrupt) + frame(2, 1020)
            + frame(2, 1020) + frame(3, 1030, invalid_measurement=True)
            + frame(4, 1040, message_id=0x52) + frame(4, 1040))
    (output / "synthetic-fixture.bin").write_bytes(wire)
    fixture = {"origin": "synthetic_from_documented_local_little_endian_profile_not_device_capture",
               "sha256": hashlib.sha256(wire).hexdigest(), "bytes": len(wire),
               "expected_decoded_frames": 6, "expected_chronology_accepted_frames": 5,
               "injections": ["noise", "overlength_header", "checksum_corruption", "duplicate", "invalid_tof_and_flow", "unknown_message_id", "embedded_EF_payload_byte"],
               "protocol_source": "https://micoair.cn/zh/docs/sensors/micolink"}
    (output / "fixture.json").write_text(json.dumps(fixture, indent=2) + "\n")
    command = [sys.executable, str(script), "--device", device, "--duration-s", "1.2",
               "--output", str(capture), "--input-origin", "synthetic_pty"]
    (output / "command.json").write_text(json.dumps(command) + "\n")
    process = None
    try:
        with (output / "capture.log").open("w") as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + 5
            while not (capture / "capture-ready.json").exists():
                if process.poll() is not None or time.monotonic() >= deadline:
                    raise RuntimeError("capture never became ready")
                time.sleep(0.01)
            # The OS may coalesce reads. Replay uses the actual recorded chunk
            # boundaries/times rather than claiming this write cadence is data
            # acquisition timing.
            offset, sizes, index = 0, (1, 2, 7, 3, 19, 5), 0
            while offset < len(wire):
                chunk = wire[offset:offset + sizes[index % len(sizes)]]
                offset += os.write(master, chunk)
                index += 1
                time.sleep(0.002)
            code = process.wait(timeout=8)
            if code != 0:
                raise RuntimeError(f"capture exited {code}")
        replay_command = [sys.executable, str(script), "--replay", str(capture)]
        replay_result = subprocess.run(replay_command, capture_output=True, text=True, timeout=8)
        (output / "replay.log").write_text(replay_result.stdout + replay_result.stderr)
        if replay_result.returncode:
            raise RuntimeError("raw replay failed")
        replay = json.loads(replay_result.stdout)
        summary = json.loads((capture / "summary.json").read_text())
        events = [json.loads(line) for line in (capture / "frames.jsonl").read_text().splitlines()]
        states = [event["chronology"] for event in events]
        unknown = [state for state in states if state.get("acquisition_valid") or state.get("metric_valid") or state.get("gyro_coverage_valid")]
        invalid_reported = [state for state in states if state.get("reported_flow_usable") is False and state.get("reported_range_usable") is False]
        checks = {
            "all_input_bytes_captured": (capture / "raw.bin").read_bytes() == wire,
            "six_supported_frames_decoded": summary["decoded_frames"] == fixture["expected_decoded_frames"],
            "duplicate_does_not_advance_chronology": summary["chronology_accepted_frames"] == fixture["expected_chronology_accepted_frames"],
            "checksum_error_recorded": summary["parser_counters"]["checksum_errors"] >= 1,
            "overlength_recorded": summary["parser_counters"]["invalid_lengths"] >= 1,
            "unknown_message_recorded": summary["parser_counters"]["unknown_message_ids"] == 1,
            "invalid_measurement_status_retained": len(invalid_reported) >= 1,
            "unknown_physical_semantics_stay_invalid": not unknown and summary["hardware_readiness"] == "NOT_READY",
            "transport_watchdog_expires_after_silence": summary["final_transport_status"]["transport_fresh"] is False,
            "no_unfinished_frame": summary["buffered_bytes_at_end"] == 0,
            "deterministic_receipt_and_byte_replay": replay["status"] == "PASS" and replay["all_raw_bytes_replayed"] == len(wire),
            "no_ap_output": summary["ap_output_messages"] == 0,
        }
        result = {"schema_version": 1, "status": "PASS" if all(checks.values()) else "FAIL",
                  "checks": checks, "fixture": fixture, "capture": summary, "replay": replay,
                  "scope": "Linux synthetic PTY and receipt replay only; no hardware, acquisition synchronization or AP ingestion"}
        (output / "result.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
        return result
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)
        os.close(master)
        os.close(slave)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = run(args.output)
    print(json.dumps({"status": result["status"], "checks": len(result["checks"]), "output": str(args.output)}))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
