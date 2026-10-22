"""Checks for release identity, complete readback, and immutable smoke scope."""
import copy
import importlib.util
import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location('ap471_gate', ROOT/'scripts/run_ap471_research_gate.py')
gate = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(gate)


def config():
    return json.loads((ROOT/'config/research_ap471_gate.json').read_text())


def rows(settings):
    return [{'param_count': len(settings), 'param_index': i, 'param_id': name, 'param_value': value}
            for i, (name, value) in enumerate(settings.items())]


def test_frozen_configuration_is_supported():
    gate.validate_config(config())


@pytest.mark.parametrize('field,value', [('hardware_access', True), ('flight_commands_allowed', True),
    ('target_system_id', 2), ('target_component_id', 191)])
def test_scope_and_identity_cannot_be_bypassed(field, value):
    settings = config()
    settings[field] = value
    with pytest.raises(ValueError):
        gate.validate_config(settings)


@pytest.mark.parametrize('field,value', [('GPS1_TYPE', 1), ('GPS2_TYPE', 1), ('SIM_GPS1_ENABLE', 1),
    ('SIM_GPS2_ENABLE', 1), ('VISO_TYPE', 2), ('PLND_ENABLED', 1), ('LOG_DISARMED', 0), ('ARMING_SKIPCHK', 1)])
def test_migration_defaults_cannot_be_bypassed(field, value):
    settings = config()
    settings['required_parameters'][field] = value
    with pytest.raises(ValueError):
        gate.validate_config(settings)


@pytest.mark.parametrize('raw,accepted', [(0x040701FF, True), (0x040507FF, False),
    (0x04070180, False), (0x040702FF, False), (True, False), (4.7, False)])
def test_live_version_requires_official_4_7_1(raw, accepted):
    assert gate.version_matches(raw, config()) is accepted


@pytest.mark.parametrize('raw,accepted', [(list(b'dbe79216'), True), (list(b'12345678'), False),
    (list(b'dbe79216\x00'), True), ([256], False), ([255], False)])
def test_live_build_prefix_must_match_pinned_commit(raw, accepted):
    assert gate.commit_prefix_matches(raw) is accepted


def test_complete_readback_requires_every_announced_index():
    settings = config()
    records = rows(settings['required_parameters'])
    assert gate.readback_result(records, settings)['status'] == 'PASS'
    assert gate.readback_result(records[:-1], settings)['complete_parameter_list'] is False


def test_required_values_alone_do_not_prove_full_parameter_list():
    settings = config()
    records = rows(settings['required_parameters'])
    for item in records:
        item['param_count'] += 100
    result = gate.readback_result(records, settings)
    assert all(result['required_checks'].values())
    assert result['status'] == 'FAIL'


def test_duplicates_do_not_fill_missing_parameter_index():
    settings = config()
    records = rows(settings['required_parameters'])
    records[-1]['param_index'] = 0
    assert gate.readback_result(records, settings)['status'] == 'FAIL'


def test_count_changes_and_value_changes_are_not_accepted():
    settings = config()
    records = rows(settings['required_parameters'])
    changed = copy.deepcopy(records)
    changed[-1]['param_count'] += 1
    assert gate.readback_result(changed, settings)['status'] == 'FAIL'
    changed = copy.deepcopy(records)
    changed[-1]['param_value'] += 1
    assert gate.readback_result(changed, settings)['status'] == 'FAIL'


@pytest.mark.parametrize('bad', [True, float('nan'), float('inf')])
def test_non_numeric_or_nonfinite_required_value_rejected(bad):
    settings = config()
    records = rows(settings['required_parameters'])
    records[0]['param_value'] = bad
    assert gate.readback_result(records, settings)['status'] == 'FAIL'


def test_fixed_length_param_id_decodes_without_rounding():
    assert gate.param_name(b'GPS1_TYPE\x00\x00') == 'GPS1_TYPE'
    assert gate.param_name('SIM_GPS1_ENABLE') == 'SIM_GPS1_ENABLE'
