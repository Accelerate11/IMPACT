"""Read-only service failures must still leave a complete original-response proof."""
import importlib.util
import itertools
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

PATH = Path(__file__).resolve().parents[1]/'query_extnav_parameter_proof.py'
SPEC = importlib.util.spec_from_file_location('query_proof', PATH)
module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(module)


def run_fake(tmp_path, monkeypatch, values, service_available=True):
    config = tmp_path/'config.json'
    config.write_text(json.dumps({'live_readiness': {'expected_system_id': 1,
        'expected_component_id': 1, 'required_parameters': {'VISO_TYPE': 2, 'PLND_ENABLED': 0, 'LOG_DISARMED': 1}}}))
    output = tmp_path/'proof.json'
    future = SimpleNamespace(done=lambda: True, result=lambda: SimpleNamespace(values=values))
    client = SimpleNamespace(wait_for_service=lambda **kwargs: service_available, call_async=lambda request: future)
    node = SimpleNamespace(create_client=lambda *args: client, destroy_node=lambda: None)
    monkeypatch.setattr(module.rclpy, 'init', lambda: None)
    monkeypatch.setattr(module.rclpy, 'create_node', lambda name: node)
    monkeypatch.setattr(module.rclpy, 'spin_until_future_complete', lambda *a, **kw: None)
    monkeypatch.setattr(module.rclpy, 'spin_once', lambda *a, **kw: None)
    monkeypatch.setattr(module.rclpy, 'ok', lambda: False)
    ticks = itertools.count(100.)
    monkeypatch.setattr(module.time, 'monotonic', lambda: next(ticks))
    code = module.main(['--config', str(config), '--output', str(output), '--timeout-s', '4'])
    return code, json.loads(output.read_text())


def value(integer, double=0.):
    return SimpleNamespace(type=2, integer_value=integer, double_value=double)


def test_matching_parameter_cache_is_a_read_only_proof(tmp_path, monkeypatch):
    code, proof = run_fake(tmp_path, monkeypatch, [value(2), value(0), value(1)])
    assert code == 0 and proof['status'] == 'PASS'
    assert proof['parameter_writes'] == proof['flight_commands'] == 0


def test_mismatched_value_preserves_actual_response_and_fails(tmp_path, monkeypatch):
    code, proof = run_fake(tmp_path, monkeypatch, [value(2), value(1), value(1)])
    assert code != 0 and proof['status'] == 'FAIL'
    assert proof['responses'][0]['values'][1]['integer_value'] == 1


def test_short_response_keeps_all_actual_values_in_valid_json(tmp_path, monkeypatch):
    code, proof = run_fake(tmp_path, monkeypatch, [value(2)])
    assert code != 0 and proof['status'] == 'FAIL'
    assert proof['responses'][0]['values'][0]['integer_value'] == 2


def test_nonfinite_unused_field_is_preserved_as_text_in_complete_fail_proof(tmp_path, monkeypatch):
    code, proof = run_fake(tmp_path, monkeypatch, [value(2, float('nan')), value(0), value(1)])
    assert code != 0 and proof['status'] == 'FAIL'
    assert proof['responses'][0]['values'][0]['double_value'] == 'nan'


def test_missing_service_writes_a_complete_fail_proof(tmp_path, monkeypatch):
    code, proof = run_fake(tmp_path, monkeypatch, [], service_available=False)
    assert code != 0 and proof['status'] == 'FAIL' and proof['responses'] == []
