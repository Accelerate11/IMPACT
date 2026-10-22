"""Pure-core tests: no ROS, aircraft, network, or old evidence is touched."""

import copy
import importlib.util
from pathlib import Path
import struct

import pytest


SOURCE = Path(__file__).resolve().parents[1] / "monitor_extnav_readiness.py"
SPEC = importlib.util.spec_from_file_location("monitor_extnav_readiness", SOURCE)
monitor = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(monitor)
NS = 1_000_000_000


def configuration():
    return copy.deepcopy(monitor.DEFAULT_CONFIG)


def proof(config=None):
    config = configuration() if config is None else config
    return {"schema_version": 1, "method": "mavros_ros2_GetParameters",
            "target_system_id": 1, "target_component_id": 1,
            "parameters": {name: {"success": True, "ros_type": 2,
                                  "integer_value": value, "double_value": 0.0}
                           for name, value in config["required_parameters"].items()}}


def core(config=None, frozen=None):
    config = configuration() if config is None else config
    return monitor.ReadinessCore(config, proof(config) if frozen is None else frozen)


def payload(present=128, enabled=128, health=128, length=31):
    raw = struct.pack("<III", present, enabled, health) + bytes(19)
    raw = raw[:length]
    raw += bytes((-len(raw)) % 8)
    return list(struct.unpack("<" + "Q" * (len(raw) // 8), raw))


def sys(c, stamp, **kwargs):
    args = {"msgid": 1, "sysid": 1, "compid": 1, "payload_length": 31,
            "payload64": payload(), "mono_ns": stamp, "wall_ns": 10 * NS + stamp}
    args.update(kwargs)
    return c.observe_sys_status(**args)


def local(c, stamp, **changes):
    status = {"local_source_ready": True, "local_clock_ready": True,
              "clock_fault_latched": False, "healthy": True}
    status.update(changes)
    return c.observe_local(status, stamp)


def fresh(c, stamp):
    local(c, stamp)
    c.observe_state(True, False, stamp)


def warm(c, start=0, end=5 * NS, step=100_000_000):
    for stamp in range(start, end + 1, step):
        fresh(c, stamp)
        sys(c, stamp)
    return c.snapshot(end)


def test_ready_exact_five_seconds_with_new_samples():
    c = core()
    assert warm(c, end=4_900_000_000)["ready"] is False
    fresh(c, 5 * NS)
    sys(c, 5 * NS)
    result = c.snapshot(5 * NS)
    assert result["ready"] is True
    assert result["healthy_sample_dwell_ns"] == 5 * NS
    assert result["ever_ready"] is True


def test_no_inputs_fails_closed():
    result = core().snapshot(0)
    assert not result["ready"]
    for reason in ("local_status_missing", "fcu_sys_status_missing", "fcu_state_missing"):
        assert reason in result["reasons"]


@pytest.mark.parametrize("field,value", [("sysid", 2), ("compid", 2),
                                        ("sysid", True), ("compid", None)])
def test_wrong_identity_clears_health(field, value):
    c = core()
    warm(c)
    result = sys(c, 5_100_000_000, **{field: value})
    assert not result["valid"]
    assert "fcu_identity_mismatch" in c.snapshot(5_100_000_000)["reasons"]
    assert c.streak_start is None


def test_other_messages_never_refresh_or_clear_sys_evidence():
    c = core()
    warm(c)
    assert sys(c, 5_100_000_000, msgid=0)["reason"] == "not_sys_status"
    assert c.sys_mono == 5 * NS
    assert c.snapshot(5_100_000_000)["ready"]
    assert not c.snapshot(5_350_000_001)["ready"]


@pytest.mark.parametrize("masks,reason", [((0, 128, 128), "fcu_vision_not_present"),
    ((128, 0, 128), "fcu_vision_not_enabled"), ((128, 128, 0), "fcu_vision_not_healthy")])
def test_health_transition_resets_dwell(masks, reason):
    c = core()
    warm(c)
    result = sys(c, 5_100_000_000, payload64=payload(*masks))
    assert result["health_transition"] == {"previous": True, "current": False}
    assert reason in c.snapshot(5_100_000_000)["reasons"]
    result = sys(c, 5_200_000_000)
    assert result["health_transition"] == {"previous": False, "current": True}
    fresh(c, 5_200_000_000)
    assert not c.snapshot(5_200_000_000)["ready"]
    assert c.streak_start == 5_200_000_000


def test_unchanged_payload_is_real_new_sample_when_mono_advances():
    c = core()
    assert warm(c)["ready"]
    assert c.counts["sys_status_events"] == 51


def test_duplicate_and_old_mono_do_not_refresh_or_add_dwell():
    c = core()
    warm(c, end=4_900_000_000)
    for stamp in (4_900_000_000, 4_800_000_000):
        assert not sys(c, stamp)["accepted"]
    fresh(c, 5 * NS)
    result = c.snapshot(5 * NS)
    assert result["healthy_sample_dwell_ns"] == 4_900_000_000
    assert not result["ready"]
    assert c.sys_mono == 4_900_000_000


def test_snapshot_cannot_finish_dwell_without_new_sys_sample():
    c = core()
    warm(c, end=4_900_000_000)
    fresh(c, 5_200_000_000)
    result = c.snapshot(5_200_000_000)
    assert not result["ready"]
    assert result["healthy_sample_dwell_ns"] == 4_900_000_000


@pytest.mark.parametrize("gap,continues", [(350_000_000, True), (350_000_001, False)])
def test_exact_gap_boundary(gap, continues):
    c = core()
    sys(c, 0)
    sys(c, gap)
    assert (c.streak_start == 0) is continues


@pytest.mark.parametrize("offset,expected", [(350_000_000, True), (350_000_001, False)])
def test_exact_sys_and_local_age_boundary(offset, expected):
    c = core()
    warm(c)
    assert c.snapshot(5 * NS + offset)["ready"] is expected


def test_stale_local_is_not_refreshed_by_fcu_or_own_snapshot():
    c = core()
    warm(c)
    for stamp in (5_100_000_000, 5_200_000_000, 5_300_000_000, 5_400_000_000):
        sys(c, stamp)
        c.observe_state(True, False, stamp)
        result = c.snapshot(stamp)
    assert "local_status_stale" in result["reasons"]
    assert not result["ready"]


def test_stale_sys_resets_and_must_rewarm():
    c = core()
    warm(c)
    fresh(c, 5_350_000_001)
    assert not c.snapshot(5_350_000_001)["ready"]
    assert c.streak_start is None
    fresh(c, 5_400_000_000)
    sys(c, 5_400_000_000)
    assert not c.snapshot(5_400_000_000)["ready"]
    assert c.streak_start == 5_400_000_000


@pytest.mark.parametrize("offset,expected", [(2 * NS, True), (2 * NS + 1, False)])
def test_state_age_boundary_independent_of_other_streams(offset, expected):
    c = core()
    warm(c)
    target = 5 * NS + offset
    for stamp in range(5_100_000_000, target + 1, 100_000_000):
        local(c, stamp)
        sys(c, stamp)
    local(c, target)
    sys(c, target)
    result = c.snapshot(target)
    assert result["ready"] is expected


@pytest.mark.parametrize("changes,reason", [({"local_source_ready": False}, "local_source_not_ready"),
    ({"local_clock_ready": False}, "local_clock_not_ready"),
    ({"healthy": False}, "local_not_healthy"),
    ({"clock_fault_latched": True}, "clock_fault_latched")])
def test_local_flags_fail_closed(changes, reason):
    c = core()
    warm(c)
    local(c, 5_100_000_000, **changes)
    assert reason in c.snapshot(5_100_000_000)["reasons"]


def test_clock_fault_sticky_even_after_later_true_health():
    c = core()
    warm(c)
    local(c, 5_100_000_000, clock_fault_latched=True)
    local(c, 5_200_000_000)
    assert "clock_fault_latched" in c.snapshot(5_200_000_000)["reasons"]


def test_replayed_clock_fault_latches_without_refreshing_receive_age():
    c = core()
    warm(c)
    assert not local(c, 5 * NS, clock_fault_latched=True)["accepted"]
    assert c.local_mono == 5 * NS
    assert "clock_fault_latched" in c.snapshot(5 * NS)["reasons"]


@pytest.mark.parametrize("status", [None, [], {}, {"healthy": True},
    {"local_source_ready": 1, "local_clock_ready": True, "clock_fault_latched": False, "healthy": True},
    {"local_source_ready": True, "local_clock_ready": True, "clock_fault_latched": False,
     "healthy": True, "extra": float("nan")}])
def test_malformed_local_invalidates_previous_good(status):
    c = core()
    warm(c)
    c.observe_local(status, 5_100_000_000)
    assert "local_status_invalid" in c.snapshot(5_100_000_000)["reasons"]


def test_disconnected_and_malformed_state():
    c = core()
    warm(c)
    c.observe_state(False, False, 5_100_000_000)
    assert "fcu_not_connected" in c.snapshot(5_100_000_000)["reasons"]
    c.observe_state(1, False, 5_200_000_000)
    assert "fcu_state_invalid" in c.snapshot(5_200_000_000)["reasons"]


def test_arm_seen_sticky_including_replayed_event():
    c = core()
    warm(c)
    c.observe_state(True, True, 5 * NS)
    assert c.armed_seen
    c.observe_state(True, False, 5_100_000_000)
    assert "armed_seen" in c.snapshot(5_100_000_000)["reasons"]


@pytest.mark.parametrize("change", ["missing_PLND", "unknown_PLND", "enabled_PLND",
    "unsuccessful", "wrong_method", "wrong_identity", "missing_target", "near_double", "nan_unused"])
def test_bad_proof_never_ready(change):
    p = proof()
    if change == "missing_PLND":
        del p["parameters"]["PLND_ENABLED"]
    elif change == "unknown_PLND":
        p["parameters"]["PLND_ENABLED"]["ros_type"] = 0
    elif change == "enabled_PLND":
        p["parameters"]["PLND_ENABLED"]["integer_value"] = 1
    elif change == "unsuccessful":
        p["parameters"]["VISO_TYPE"]["success"] = False
    elif change == "wrong_method":
        p["method"] = "ParamGet"
    elif change == "wrong_identity":
        p["target_system_id"] = 2
    elif change == "missing_target":
        del p["target_component_id"]
    elif change == "near_double":
        p["parameters"]["VISO_TYPE"].update(ros_type=3, double_value=1.999999999)
    elif change == "nan_unused":
        p["parameters"]["VISO_TYPE"]["double_value"] = float("nan")
    c = core(frozen=p)
    result = warm(c)
    assert not result["parameter_proof_valid"]
    assert "parameter_proof_invalid" in result["reasons"]


def test_exact_double_proof_allowed_without_rounding():
    p = proof()
    for row in p["parameters"].values():
        row.update(ros_type=3, double_value=float(row["integer_value"]))
    assert warm(core(frozen=p))["ready"]


@pytest.mark.parametrize("kwargs", [{"framing_status": 2}, {"payload_length": 0},
    {"payload_length": 32}, {"payload64": []}, {"payload64": [1]},
    {"payload64": [float("nan")] * 4}, {"payload64": [-1] * 4},
    {"payload64": [2**64] * 4}, {"magic": 99}, {"payload_length": 12, "magic": 254}])
def test_malformed_sys_clears_previous_health(kwargs):
    c = core()
    warm(c)
    result = sys(c, 5_100_000_000, **kwargs)
    assert not result["valid"]
    assert c.streak_start is None
    assert not c.snapshot(5_100_000_000)["ready"]


def test_mavlink2_tail_zero_truncation_preserves_health_decode():
    decoded = monitor.decode_sys_status(9, payload(length=9), magic=253)
    assert decoded["health_mask"] == 128
    assert decoded["trailing_zero_extension"] == 3
    assert warm(core())["ready"]


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -1, 0, True, 0.0000000001])
def test_invalid_time_parameters_rejected(value):
    config = configuration()
    config["sys_status_max_age_s"] = value
    with pytest.raises(ValueError):
        core(config)


@pytest.mark.parametrize("key,value", [("vision_position_mask", 64), ("expected_system_id", 2),
    ("check_rate_hz", 1), ("schema_version", True), ("required_parameters", {"VISO_TYPE": 2})])
def test_unsafe_or_unknown_config_rejected(key, value):
    config = configuration()
    config[key] = value
    with pytest.raises(ValueError):
        core(config)


def test_wrapped_configuration_and_integer_ns_at_epoch_scale():
    config = configuration()
    c = core({"live_readiness": config}, proof(config))
    start = 1_800_000_000_000_000_000
    assert warm(c, start=start, end=start + 5 * NS)["ready"]
    assert c.snapshot(start + 5 * NS + 350_000_001)["ready"] is False


def test_local_and_state_replay_do_not_refresh_age():
    c = core()
    warm(c)
    assert not local(c, 5 * NS)["accepted"]
    assert not c.observe_state(True, False, 5 * NS)["accepted"]
    assert c.local_mono == c.state_mono == 5 * NS


def test_monitor_monotonic_regression_is_latched():
    c = core()
    warm(c)
    assert "monitor_monotonic_clock_fault" in c.snapshot(5 * NS - 1)["reasons"]
    assert "monitor_monotonic_clock_fault" in c.snapshot(5 * NS)["reasons"]


@pytest.mark.parametrize("timestamp", [float("nan"), float("inf"), True, -1, 1.5])
def test_invalid_timestamp_is_not_silently_coerced(timestamp):
    c = core()
    with pytest.raises(ValueError):
        sys(c, timestamp)
    with pytest.raises(ValueError):
        c.observe_local({}, 0, wall_ns=timestamp)
    with pytest.raises(ValueError):
        c.observe_state(True, False, 0, wall_ns=timestamp)
    with pytest.raises(ValueError):
        sys(c, 0, wall_ns=timestamp)


def test_no_flight_or_parameter_service_api_in_monitor_ast():
    import ast
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    attributes = {node.attr for node in ast.walk(tree) if isinstance(node, ast.Attribute)}
    assert "create_client" not in attributes
    publishers = [node for node in ast.walk(tree) if isinstance(node, ast.Call)
                  and isinstance(node.func, ast.Attribute) and node.func.attr == "create_publisher"]
    assert len(publishers) == 1
    assert publishers[0].args[1].value == "/impact/extnav/readiness"
