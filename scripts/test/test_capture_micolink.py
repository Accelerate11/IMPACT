"""Synthetic archive integrity and deterministic receipt replay, without a TTY.

Parser details are covered independently by test_micolink.py. These fixtures
model captured files solely to test the raw-byte index and evidence boundaries;
they are explicitly synthetic and establish no hardware timing or calibration.
"""

from copy import deepcopy
import hashlib
import json
from pathlib import Path
import runpy

import pytest

pytest.importorskip("termios")

from xq_autonomy.micolink import MicolinkChronologyGuard, MicolinkParser


ROOT = Path(__file__).resolve().parents[2]
CAPTURE = runpy.run_path(str(ROOT / "scripts/capture_micolink.py"))
REPLAY = CAPTURE["replay"]
FILES = ("raw.bin", "chunks.jsonl", "frames.jsonl")

# Hand-assembled synthetic packets: sequence FE then FF, counter advances 10ms.
FIRST = bytes.fromhex("EF0F0051FE14 04030201 E8030000 28020100 85FFC801 C8010000 97")
SECOND = bytes.fromhex("EF0F0051FF14 0E030201 E8030000 28020100 85FFC801 C8010000 A2")


def write_json(path, value):
    path.write_text(json.dumps(value, allow_nan=False) + "\n")


def write_lines(path, values):
    path.write_text("".join(json.dumps(value, allow_nan=False) + "\n" for value in values))


def file_record(path):
    data = path.read_bytes()
    return {"size_bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()}


def resign(directory, name):
    """Let semantic-index tests pass the outer byte-integrity check deliberately."""
    summary = json.loads((directory / "summary.json").read_text())
    summary["files"][name] = file_record(directory / name)
    write_json(directory / "summary.json", summary)


def archive(tmp_path):
    directory = tmp_path / "synthetic_receipt_replay"
    directory.mkdir()
    chunks = (b"\x00\x11\x22" + FIRST[:9], FIRST[9:] + FIRST + SECOND)
    parser = MicolinkParser()
    guard = MicolinkChronologyGuard()
    guard.new_session(directory.name)
    index, frames, offset = [], [], 0
    for data, receipt in zip(chunks, (10.0, 10.01)):
        index.append({"offset": offset, "length": len(data), "receipt_monotonic_s": receipt})
        offset += len(data)
        for message in parser.feed(data):
            state = guard.observe(message, receipt_monotonic_s=receipt, now_monotonic_s=receipt)
            frames.append({"message": message.to_dict(), "receipt_monotonic_s": receipt,
                           "chronology": state})
    (directory / "raw.bin").write_bytes(b"".join(chunks))
    write_lines(directory / "chunks.jsonl", index)
    write_lines(directory / "frames.jsonl", frames)
    write_json(directory / "summary.json", {
        "schema_version": 1, "status": "CAPTURED", "input_origin": "synthetic_pty",
        "parser_counters": dict(parser.counters), "buffered_bytes_at_end": parser.buffered_bytes,
        "files": {name: file_record(directory / name) for name in FILES},
    })
    return directory, index, frames


def test_replay_covers_raw_chunks_rejects_duplicate_and_preserves_original_artifacts(tmp_path):
    directory, _, frames = archive(tmp_path)
    assert [frame["chronology"]["accepted"] for frame in frames] == [True, False, True]
    before = {name: (directory / name).read_bytes() for name in (*FILES, "summary.json")}
    result = REPLAY(directory)
    assert result["status"] == "PASS"
    assert result["frames"] == 3
    assert result["all_raw_bytes_replayed"] == len(before["raw.bin"])
    assert result["input_origin"] == "synthetic_pty"
    assert before == {name: (directory / name).read_bytes() for name in before}


@pytest.mark.parametrize("name", FILES)
def test_any_raw_chunk_or_decoded_evidence_tamper_is_rejected_by_hash(tmp_path, name):
    directory, _, _ = archive(tmp_path)
    path = directory / name
    data = bytearray(path.read_bytes())
    data[0] ^= 1
    path.write_bytes(data)
    with pytest.raises(ValueError, match="hash mismatch"):
        REPLAY(directory)


@pytest.mark.parametrize("problem", ["first_offset", "overlap", "overrun", "zero_length", "missing_tail"])
def test_even_resigned_chunk_index_must_cover_each_raw_byte_exactly_once(tmp_path, problem):
    directory, index, _ = archive(tmp_path)
    broken = deepcopy(index)
    if problem == "first_offset":
        broken[0]["offset"] = 1
    elif problem == "overlap":
        broken[1]["offset"] -= 1
    elif problem == "overrun":
        broken[1]["length"] += 1
    elif problem == "zero_length":
        broken[0]["length"] = 0
    elif problem == "missing_tail":
        broken.pop()
    write_lines(directory / "chunks.jsonl", broken)
    resign(directory, "chunks.jsonl")
    with pytest.raises(ValueError, match="chunk index"):
        REPLAY(directory)


def test_hash_manifest_cannot_exclude_raw_bytes_that_only_contain_noise(tmp_path):
    directory, _, _ = archive(tmp_path)
    summary = json.loads((directory / "summary.json").read_text())
    del summary["files"]["raw.bin"]
    write_json(directory / "summary.json", summary)
    raw = bytearray((directory / "raw.bin").read_bytes())
    raw[0] = 0x33  # Parser results/counters stay identical; the raw hash must catch this.
    (directory / "raw.bin").write_bytes(raw)
    with pytest.raises(ValueError):
        REPLAY(directory)


def test_replay_recomputes_duplicate_rejection_instead_of_trusting_resigned_frames(tmp_path):
    directory, _, frames = archive(tmp_path)
    frames[1]["chronology"]["accepted"] = True
    write_lines(directory / "frames.jsonl", frames)
    resign(directory, "frames.jsonl")
    result = REPLAY(directory)
    assert result["status"] == "FAIL"


def test_capture_refuses_to_replace_existing_evidence_before_opening_any_device(tmp_path):
    directory, _, _ = archive(tmp_path)
    marker = (directory / "raw.bin").read_bytes()
    with pytest.raises(ValueError, match="output must be new"):
        CAPTURE["capture"](Path("/nonexistent/synthetic-test-device"), directory, 0.1, "synthetic_pty")
    assert (directory / "raw.bin").read_bytes() == marker
