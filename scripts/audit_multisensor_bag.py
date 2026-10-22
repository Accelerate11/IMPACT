#!/usr/bin/env python3
"""Postflight camera evidence audit; never decompress or open SQLite in the source bag.

Run with the installed ROS Python environment and --run-dir CLOSED_TRIAL.  The
only persistent output is CLOSED_TRIAL/camera-bag-audit.json.  This additional
storage/calibration audit does not replace the frozen numerical acceptance gate.
FILE/zstd is independently streamed through system libzstd into a private temp
directory; uncompressed SQLite chunks and existing sidecars are also supported.
Python SQLite opens only the private copy in read-only mode (immutable without
WAL).  No new
packages, rosbag replay, raw-bag repairs, or modification of raw data are used.
"""

from __future__ import annotations

import argparse
import ctypes
import ctypes.util
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import sqlite3
import tempfile
import time


STATUS_TOPIC = "/impact/r2/camera_bridge/status"
CALIBRATION_TOPIC = "/impact/r2/calibration_metadata"
TARGET_TOPICS = (STATUS_TOPIC, CALIBRATION_TOPIC)
SAMPLE_TOPICS = ("/clock", "/livox/imu", "/impact/r2/front/image_raw", "/impact/r2/down/image_raw")


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def snapshot(directory):
    result = {}
    for path in sorted(directory.rglob("*")):
        if path.is_symlink():
            raise ValueError(f"source bag contains a symlink: {path}")
        if path.is_file():
            result[path.relative_to(directory).as_posix()] = {
                "bytes": path.stat().st_size, "sha256": sha256(path)
            }
    return result


def decompress_zstd(source, destination):
    """Bounded-memory independent zstd decoding, including concatenated frames."""
    class Input(ctypes.Structure):
        _fields_ = [("src", ctypes.c_void_p), ("size", ctypes.c_size_t), ("pos", ctypes.c_size_t)]

    class Output(ctypes.Structure):
        _fields_ = [("dst", ctypes.c_void_p), ("size", ctypes.c_size_t), ("pos", ctypes.c_size_t)]

    library = ctypes.util.find_library("zstd")
    if not library:
        raise RuntimeError("system libzstd unavailable; no packages are installed by this audit")
    zstd = ctypes.CDLL(library)
    zstd.ZSTD_createDStream.restype = ctypes.c_void_p
    zstd.ZSTD_freeDStream.argtypes = [ctypes.c_void_p]
    zstd.ZSTD_initDStream.argtypes = [ctypes.c_void_p]
    zstd.ZSTD_initDStream.restype = ctypes.c_size_t
    zstd.ZSTD_decompressStream.argtypes = [ctypes.c_void_p, ctypes.POINTER(Output), ctypes.POINTER(Input)]
    zstd.ZSTD_decompressStream.restype = ctypes.c_size_t
    zstd.ZSTD_isError.argtypes = [ctypes.c_size_t]
    zstd.ZSTD_isError.restype = ctypes.c_uint
    zstd.ZSTD_getErrorName.argtypes = [ctypes.c_size_t]
    zstd.ZSTD_getErrorName.restype = ctypes.c_char_p

    def checked(code):
        if zstd.ZSTD_isError(code):
            raise RuntimeError("zstd: " + zstd.ZSTD_getErrorName(code).decode())
        return code

    stream = zstd.ZSTD_createDStream()
    if not stream:
        raise RuntimeError("cannot allocate zstd stream")
    written, frames, remaining = 0, 0, None
    try:
        checked(zstd.ZSTD_initDStream(stream))
        out_buffer = ctypes.create_string_buffer(1024 * 1024)
        with source.open("rb") as compressed, destination.open("xb") as decoded:
            for chunk in iter(lambda: compressed.read(1024 * 1024), b""):
                in_buffer = ctypes.create_string_buffer(chunk)
                incoming = Input(ctypes.cast(in_buffer, ctypes.c_void_p), len(chunk), 0)
                while True:
                    outgoing = Output(ctypes.cast(out_buffer, ctypes.c_void_p), len(out_buffer), 0)
                    old_pos = incoming.pos
                    remaining = checked(zstd.ZSTD_decompressStream(stream, ctypes.byref(outgoing), ctypes.byref(incoming)))
                    decoded.write(out_buffer.raw[:outgoing.pos])
                    written += outgoing.pos
                    if remaining == 0:
                        frames += 1
                    if incoming.pos == old_pos and outgoing.pos == 0:
                        if incoming.pos == incoming.size:
                            break
                        raise RuntimeError("zstd decoder made no progress")
                    # A full output buffer may need draining even after all
                    # supplied compressed input was consumed.  Do not request a
                    # new frame after a completed frame merely to drain it.
                    if incoming.pos == incoming.size and (outgoing.pos < outgoing.size or remaining == 0):
                        break
    finally:
        zstd.ZSTD_freeDStream(stream)
    complete = remaining == 0 and frames > 0
    return {"status": "PASS" if complete else "FAIL", "decoder": library,
            "bytes": written, "frames": frames, "last_decoder_return": remaining,
            "error": None if complete else "truncated or empty zstd frame"}


def inspect_database(path, topic_results, deserialize, get_message):
    report = {"file": path.name, "status": "FAIL", "errors": [], "payload_samples": []}
    counts = {}
    with path.open("rb") as source:
        header = source.read(100)
    if len(header) >= 100:
        page_size = int.from_bytes(header[16:18], "big")
        page_size = 65536 if page_size == 1 else page_size
        report["sqlite_header"] = {
            "signature_valid": header[:16] == b"SQLite format 3\x00",
            "page_size_bytes": page_size,
            "header_page_count": int.from_bytes(header[28:32], "big"),
            "decoded_size_bytes": path.stat().st_size,
            "size_mod_page": path.stat().st_size % page_size if page_size else None,
        }
    has_wal = path.with_name(path.name + "-wal").is_file()
    report["sqlite_open_mode"] = "mode=ro" if has_wal else "mode=ro&immutable=1"
    connection = sqlite3.connect(path.as_uri() + "?" + report["sqlite_open_mode"], uri=True)
    try:
        connection.execute("PRAGMA query_only=ON")
        for pragma in ("quick_check", "integrity_check"):
            try:
                values = [row[0] for row in connection.execute(f"PRAGMA {pragma}")]
                report[pragma] = values
                if values != ["ok"]:
                    report["errors"].append(f"SQLite {pragma} did not return only ok")
            except sqlite3.DatabaseError as exc:
                report[pragma] = [str(exc)]
                report["errors"].append(f"SQLite {pragma}: {exc}")
        try:
            topics = {
                row[0]: {"name": row[1], "type": row[2], "serialization_format": row[3]}
                for row in connection.execute("SELECT id, name, type, serialization_format FROM topics")
            }
            report["topics"] = topics
        except sqlite3.DatabaseError as exc:
            report["errors"].append(f"topics unreadable: {exc}")
            return report, counts
        try:
            # NOT INDEXED forces a complete messages-table scan; metadata is not
            # treated as proof that all messages were committed and are readable.
            for topic_id, count in connection.execute("SELECT topic_id, COUNT(*) FROM messages NOT INDEXED GROUP BY topic_id"):
                if topic_id not in topics:
                    report["errors"].append(f"messages refer to unknown topic id {topic_id}")
                else:
                    counts[topics[topic_id]["name"]] = count
            report["message_count"] = sum(counts.values())
            report["message_counts_by_topic"] = counts
        except sqlite3.DatabaseError as exc:
            report["errors"].append(f"full message count scan: {exc}")
        try:
            payload_hash = hashlib.sha256()
            payload_rows, payload_bytes, payload_counts = 0, 0, {}
            for message_id, topic_id, timestamp, data in connection.execute(
                "SELECT id, topic_id, timestamp, data FROM messages NOT INDEXED ORDER BY id"
            ):
                if not isinstance(data, bytes):
                    raise ValueError(f"message {message_id} payload is not a BLOB")
                if topic_id not in topics:
                    raise ValueError(f"message {message_id} has unknown topic id")
                # Read every overflow BLOB, not just table row counts.  Include
                # unambiguous row metadata in the reproducible ordered digest.
                header = json.dumps([message_id, topic_id, timestamp, len(data)], separators=(",", ":")).encode("ascii")
                payload_hash.update(header + b"\n")
                payload_hash.update(data)
                payload_hash.update(b"\n")
                payload_rows += 1
                payload_bytes += len(data)
                topic_name = topics[topic_id]["name"]
                payload_counts[topic_name] = payload_counts.get(topic_name, 0) + 1
            matches = payload_counts == counts and payload_rows == sum(counts.values())
            report["full_serialized_payload_read"] = {
                "status": "PASS" if matches else "FAIL", "rows": payload_rows,
                "serialized_bytes": payload_bytes, "ordered_row_payload_sha256": payload_hash.hexdigest(),
                "topic_counts": payload_counts, "counts_match_independent_count_scan": matches,
                "semantic_deserialization": "all target JSON records; representative sensor samples only",
            }
            if not matches:
                report["errors"].append("full payload read counts differ from independent COUNT scan")
        except Exception as exc:
            report["full_serialized_payload_read"] = {"status": "FAIL", "error": f"{type(exc).__name__}: {exc}"}
            report["errors"].append(f"full serialized payload scan: {exc}")
        string_class = get_message("std_msgs/msg/String")
        for topic_id, topic in topics.items():
            name = topic["name"]
            if name not in TARGET_TOPICS:
                continue
            record = topic_results[name]
            if topic["type"] != "std_msgs/msg/String" or topic["serialization_format"] != "cdr":
                report["errors"].append(f"target topic has unexpected serialization/type: {name}")
                continue
            try:
                for message_id, timestamp, data in connection.execute(
                    "SELECT id, timestamp, data FROM messages NOT INDEXED WHERE topic_id=? ORDER BY id", (topic_id,)
                ):
                    decoded = json.loads(deserialize(data, string_class).data)
                    if not isinstance(decoded, dict):
                        raise ValueError("expected JSON object")
                    record["read_count"] += 1
                    if timestamp >= record.get("last_bag_timestamp_ns", -1):
                        record.update(last_bag_timestamp_ns=timestamp, last_message_id=message_id, last_json=decoded)
            except Exception as exc:
                report["errors"].append(f"target JSON read {name}: {type(exc).__name__}: {exc}")
        # Deserialize representative sensor payloads as well as all target String
        # records; this is a sample, not verification of every sensor payload.
        for name in SAMPLE_TOPICS:
            found = [(topic_id, topic) for topic_id, topic in topics.items() if topic["name"] == name]
            if not found:
                continue
            topic_id, topic = found[0]
            sample = {"topic": name, "type": topic["type"], "status": "FAIL"}
            try:
                row = connection.execute("SELECT timestamp, data FROM messages NOT INDEXED WHERE topic_id=? LIMIT 1", (topic_id,)).fetchone()
                if row is None:
                    continue  # A chunk may contain no messages of this topic.
                message = deserialize(row[1], get_message(topic["type"]))
                sample.update(status="PASS", timestamp_ns=row[0], serialized_bytes=len(row[1]))
                if hasattr(message, "width"):
                    sample.update(width=message.width, height=message.height, encoding=message.encoding)
            except Exception as exc:
                sample["error"] = f"{type(exc).__name__}: {exc}"
                report["errors"].append(f"sample deserialize failed: {name}: {exc}")
            report["payload_samples"].append(sample)
        report["status"] = "PASS" if not report["errors"] else "FAIL"
    finally:
        connection.close()
    return report, counts


def audit(run):
    started = time.monotonic()
    result = {
        "schema_version": 1, "status": "FAIL", "scope": "additional_postflight_storage_and_camera_calibration_audit",
        "numerical_gate_thresholds_modified": False, "run_dir": str(run),
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "script_path": str(Path(__file__).resolve()), "script_sha256": sha256(Path(__file__).resolve()),
        "method": "private full raw copy; independent libzstd when compressed; read-only SQLite",
        "checks": {}, "errors": [], "databases": [],
        "topic_results": {name: {"read_count": 0} for name in TARGET_TOPICS},
    }
    bag = run / "rosbag"
    before = None
    checks = result["checks"]
    try:
        if not (run / "runner-result.json").is_file() or not (run / "shutdown-result.json").is_file():
            raise ValueError("closed trial required: runner-result.json and shutdown-result.json must exist")
        checks["closed_trial_artifacts_present"] = True
        before = snapshot(bag)
        result["original_files_before"] = before
        import yaml
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        try:
            import rosbag2_py
            result["sequential_compression_reader_available"] = hasattr(rosbag2_py, "SequentialCompressionReader")
        except ImportError:
            result["sequential_compression_reader_available"] = False
        metadata = yaml.safe_load((bag / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
        compressed = metadata.get("compression_format", "").lower() == "zstd" and metadata.get("compression_mode", "").lower() == "file"
        uncompressed = metadata.get("compression_format", "").lower() in ("", "none") and metadata.get("compression_mode", "").lower() in ("", "none")
        if metadata["storage_identifier"] != "sqlite3" or not (compressed or uncompressed):
            raise ValueError("supported storage: sqlite3, uncompressed or FILE/zstd")
        result["compression"] = "FILE/zstd" if compressed else "none"
        expected = {}
        for item in metadata["topics_with_message_count"]:
            name, count = item["topic_metadata"]["name"], item["message_count"]
            if name in expected or type(count) is not int or count < 0:
                raise ValueError("duplicate topic or invalid metadata message count")
            expected[name] = count
        expected_total = metadata["message_count"]
        if type(expected_total) is not int or expected_total <= 0:
            raise ValueError("metadata requires a positive total message count")
        result["expected_message_count"] = expected_total
        result["metadata_topic_counts"] = expected
        checks["metadata_counts_self_consistent"] = sum(expected.values()) == expected_total
        raw_names = metadata["relative_file_paths"]
        if not raw_names or len(set(raw_names)) != len(raw_names):
            raise ValueError("missing or repeated compressed paths")
        suffix = ".db3.zstd" if compressed else ".db3"
        observed_raw = {name for name in before if name.endswith((".db3", ".db3.zstd"))}
        checks["raw_file_set_matches_metadata"] = observed_raw == set(raw_names)
        file_metadata = {entry["path"]: entry for entry in metadata.get("files", [])}
        aggregate = {}
        with tempfile.TemporaryDirectory(prefix="impact-r2-bag-audit-") as temporary:
            private = Path(temporary)
            result["temporary_directory"] = str(private)
            for name in raw_names:
                relative = Path(name)
                if relative.is_absolute() or ".." in relative.parts or not name.endswith(suffix):
                    raise ValueError("refusing nonlocal/non-db3 compressed metadata path")
                source, copied = bag / relative, private / relative
                copied.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(source, copied)
                if sha256(copied) != before[relative.as_posix()]["sha256"]:
                    raise ValueError("compressed private copy SHA256 does not match original")
                decoded = copied.with_suffix("") if compressed else copied
                sidecars = []
                if not compressed:
                    for ending in ("-wal", "-shm", "-journal"):
                        source_sidecar = source.with_name(source.name + ending)
                        if source_sidecar.is_file():
                            target_sidecar = copied.with_name(copied.name + ending)
                            shutil.copy2(source_sidecar, target_sidecar)
                            if sha256(target_sidecar) != before[(relative.parent / (relative.name + ending)).as_posix()]["sha256"]:
                                raise ValueError("sidecar private copy SHA256 mismatch")
                            sidecars.append(source_sidecar.name)
                database = {"source_raw_file": name, "status": "FAIL", "errors": []}
                try:
                    decompression = decompress_zstd(copied, decoded) if compressed else {"status": "PASS", "mode": "none", "bytes": decoded.stat().st_size}
                    database, counts = inspect_database(decoded, result["topic_results"], deserialize_message, get_message)
                    database.update(source_raw_file=name, decompression=decompression, copied_sidecars=sidecars)
                    if decompression["status"] != "PASS":
                        database["status"] = "FAIL"
                        database["errors"].append(decompression["error"])
                    declared_file = file_metadata.get(name, file_metadata.get(name.removesuffix(".zstd")))
                    if declared_file is not None:
                        declared_count = declared_file.get("message_count")
                        database["metadata_message_count"] = declared_count
                        database["metadata_message_count_matches"] = type(declared_count) is int and declared_count == sum(counts.values())
                        if not database["metadata_message_count_matches"]:
                            database["status"] = "FAIL"
                            database["errors"].append("per-file metadata message count mismatch")
                    for topic, count in counts.items():
                        aggregate[topic] = aggregate.get(topic, 0) + count
                except Exception as exc:
                    database["errors"].append(f"{type(exc).__name__}: {exc}")
                result["databases"].append(database)
            result["temporary_files_before_cleanup"] = sorted(p.relative_to(private).as_posix() for p in private.rglob("*") if p.is_file())
        result["temporary_copy_cleaned"] = not Path(result["temporary_directory"]).exists()
        checks["temporary_copy_cleaned"] = result["temporary_copy_cleaned"]
        checks["all_database_audits_pass"] = bool(result["databases"]) and all(db["status"] == "PASS" for db in result["databases"])
        checks["all_sqlite_integrity_checks_pass"] = bool(result["databases"]) and all(
            db.get("quick_check") == ["ok"] and db.get("integrity_check") == ["ok"]
            for db in result["databases"]
        )
        checks["all_source_compression_frames_complete"] = bool(result["databases"]) and all(
            db.get("decompression", {}).get("status") == "PASS" for db in result["databases"]
        )
        checks["all_serialized_payloads_read"] = bool(result["databases"]) and all(
            db.get("full_serialized_payload_read", {}).get("status") == "PASS" for db in result["databases"]
        )
        successful_samples = {sample["topic"] for db in result["databases"] for sample in db.get("payload_samples", []) if sample["status"] == "PASS"}
        checks["all_representative_payload_types_deserialized"] = set(SAMPLE_TOPICS).issubset(successful_samples)
        result["actual_message_count"] = sum(aggregate.values())
        result["actual_topic_counts"] = aggregate
        checks["all_topic_counts_match_metadata"] = all(aggregate.get(name, 0) == count for name, count in expected.items()) and set(aggregate).issubset(expected)
        checks["total_message_count_matches_metadata"] = result["actual_message_count"] == expected_total
        for name in TARGET_TOPICS:
            record = result["topic_results"][name]
            record["expected_count"] = expected.get(name, 0)
            checks[f"target_complete:{name}"] = record["read_count"] == record["expected_count"] > 0
        status = result["topic_results"][STATUS_TOPIC].get("last_json", {})
        calibration = result["topic_results"][CALIBRATION_TOPIC].get("last_json", {})
        checks["status_does_not_use_truth"] = status.get("ground_truth_used") is False
        result["fallback_counts"] = {}
        for camera in ("front", "down"):
            stream = status.get("cameras", {}).get(camera, {})
            checks[f"{camera}_native_info_seen"] = stream.get("native_info_seen") is True
            checks[f"{camera}_calibration_unblocked"] = stream.get("calibration_blocked") is False
            for counter in ("native_infos_rejected", "images_rejected"):
                checks[f"{camera}_{counter}_zero"] = type(stream.get(counter)) is int and stream[counter] == 0
            fallback = stream.get("fallback_infos_published")
            checks[f"{camera}_fallback_count_reported"] = type(fallback) is int and fallback >= 0
            result["fallback_counts"][camera] = fallback
        for field, value in (("ground_truth_used", False), ("pixel_payload_modified", False), ("simulation_calibration", True), ("real_camera_calibration", False)):
            checks[f"metadata_{field}"] = calibration.get(field) is value
    except Exception as exc:
        result["errors"].append(f"{type(exc).__name__}: {exc}")
    finally:
        try:
            after = snapshot(bag)
            result["original_files_after"] = after
            checks["original_bag_file_set_and_sha256_unchanged"] = before is not None and before == after
        except Exception as exc:
            checks["original_bag_file_set_and_sha256_unchanged"] = False
            result["errors"].append(f"after snapshot: {type(exc).__name__}: {exc}")
        result["duration_wall_s"] = round(time.monotonic() - started, 6)
    result["failed_checks"] = [name for name, valid in checks.items() if valid is not True]
    result["status"] = "PASS" if checks and not result["failed_checks"] and not result["errors"] else "FAIL"
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    run = args.run_dir.resolve()
    if not run.is_dir():
        parser.error("--run-dir must be an existing closed trial directory")
    result = audit(run)
    target = run / "camera-bag-audit.json"
    temporary = target.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False) + "\n")
    temporary.replace(target)
    print(json.dumps({"status": result["status"], "artifact": str(target), "failed_checks": result["failed_checks"], "errors": result["errors"], "actual_message_count": result.get("actual_message_count"), "fallback_counts": result.get("fallback_counts")}, ensure_ascii=False))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
