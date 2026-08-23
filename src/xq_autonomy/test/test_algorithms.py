import math
import json
from pathlib import Path

import numpy as np
import pytest

from xq_autonomy.exploration import OAER2D
from xq_autonomy.localization import DafLioProxy2D
from xq_autonomy.mapping import TDSemMap2D
from xq_autonomy.metrics import (
    append_state_transition,
    fault_evidence_summary,
    frequency_metrics,
    replan_metrics,
    trajectory_metrics,
)
from xq_autonomy.network import GroundLinkHeartbeatRelay, PacketLossRelay
from xq_autonomy.planning import (
    R2EgoProxy2D,
    adaptive_safe_radius,
    brake_distance,
    rate_limit_due,
)
from xq_autonomy.sentinel import SentinelFSM
from xq_autonomy.types import AutonomyMode, HealthLevel, HealthSample, Packet, Pose2D


def _critical_health(fsm: SentinelFSM) -> None:
    for module in ("fcu", "localization", "lidar", "planner"):
        fsm.set_health(module, HealthSample(HealthLevel.OK, 0.01, 1.0, ""))


def test_daf_proxy_detects_directional_degeneracy() -> None:
    proxy = DafLioProxy2D(seed=7)
    angles = np.linspace(-math.pi, math.pi, 720, endpoint=False)
    balanced = np.column_stack([5.0 * np.cos(angles), 5.0 * np.sin(angles)])
    balanced_quality = proxy.observe(balanced)
    corridor = np.column_stack([np.linspace(-8.0, 8.0, 720), np.ones(720) * 1.2])
    corridor_quality = proxy.observe(corridor)
    assert balanced_quality.degeneracy_score < 0.05
    assert corridor_quality.degeneracy_score > balanced_quality.degeneracy_score + 0.20
    assert corridor_quality.effective_points == 720


def test_map_is_bounded_and_frontiers_exist() -> None:
    mapping = TDSemMap2D(width_m=8.0, height_m=8.0, resolution_m=0.10)
    pose = Pose2D()
    angles = np.linspace(-math.pi, math.pi, 180, endpoint=False)
    ring = np.column_stack([2.0 * np.cos(angles), 2.0 * np.sin(angles)])
    hits = mapping.update(pose, ring, now_s=1.0)
    assert hits == 180
    assert 0.0 < mapping.known_fraction() < 1.0
    assert mapping.frontier_cells()
    assert mapping.log_odds.shape == (80, 80)


def test_dynamic_ttl_restores_transient_cell() -> None:
    mapping = TDSemMap2D(width_m=4.0, height_m=4.0, resolution_m=0.10, dynamic_ttl_s=1.0)
    gx, gy = mapping.world_to_grid(0.8, 0.0)
    mapping.observed[gy, gx] = True
    mapping.log_odds[gy, gx] = 2.0
    mapping.dynamic_confidence[gy, gx] = 0.14
    mapping.last_seen_s[gy, gx] = 0.0
    mapping.decay(2.0)
    assert mapping.dynamic_confidence[gy, gx] == 0.0
    assert mapping.log_odds[gy, gx] < 0.0


def test_oaer_scores_frontier_and_observability() -> None:
    mapping = TDSemMap2D(width_m=6.0, height_m=6.0, resolution_m=0.10)
    cx, cy = mapping.world_to_grid(0.0, 0.0)
    mapping.observed[cy - 10 : cy + 11, cx - 10 : cx + 11] = True
    mapping.log_odds[cy - 10 : cy + 11, cx - 10 : cx + 11] = -1.0
    scores = OAER2D().score_candidates(mapping, Pose2D(), weak_direction=[1.0, 0.0])
    assert scores
    assert scores == sorted(scores, key=lambda item: item.utility, reverse=True)
    assert all(0.0 <= item.observability <= 1.0 for item in scores)


def test_oaer_rejects_frontiers_outside_safe_reachable_component() -> None:
    mapping = TDSemMap2D(width_m=6.0, height_m=6.0, resolution_m=0.10)
    cx, cy = mapping.world_to_grid(0.0, 0.0)
    mapping.observed[cy - 10 : cy + 11, cx - 10 : cx + 11] = True
    mapping.log_odds[cy - 10 : cy + 11, cx - 10 : cx + 11] = -1.0
    reachable = np.zeros_like(mapping.observed)
    reachable[:, :cx] = True

    scores = OAER2D().score_candidates(
        mapping,
        Pose2D(),
        weak_direction=[1.0, 0.0],
        reachable_mask=reachable,
    )

    assert scores
    for item in scores:
        gx, gy = mapping.world_to_grid(*item.goal)
        assert reachable[gy, gx]


def test_adaptive_radius_is_monotonic() -> None:
    low = adaptive_safe_radius(0.25, 0.10, np.eye(2) * 0.001, 0.5, 0.05)
    high_cov = adaptive_safe_radius(0.25, 0.10, np.eye(2) * 0.040, 0.5, 0.05)
    high_speed = adaptive_safe_radius(0.25, 0.10, np.eye(2) * 0.040, 2.0, 0.20)
    assert low < high_cov < high_speed
    assert brake_distance(2.0, 2.0, 0.1, 0.2) > 1.0


def test_planner_accepts_path_and_deadline_brakes() -> None:
    occupancy = np.zeros((60, 60), dtype=np.int8)
    occupancy[10:50, 30] = 100
    occupancy[28:33, 30] = 0
    planner = R2EgoProxy2D(resolution_m=0.1, deadline_s=0.2)
    accepted = planner.plan(occupancy, (5, 30), (55, 30), safe_radius_m=0.0)
    assert accepted.accepted
    assert accepted.path
    timed_out = planner.plan(
        occupancy,
        (5, 30),
        (55, 30),
        safe_radius_m=0.0,
        forced_delay_s=0.25,
    )
    assert not timed_out.accepted
    assert timed_out.brake_fallback
    assert timed_out.reason == "deadline_exceeded"


def test_planner_adjusts_inflated_goal_within_reachable_component() -> None:
    occupancy = np.zeros((21, 31), dtype=np.int8)
    occupancy[:, 15] = 100
    planner = R2EgoProxy2D(resolution_m=0.1, deadline_s=0.2)

    # The requested cell is inside the inflated wall.  Cells just to its right
    # are closer in Euclidean distance but unreachable from the start.
    result = planner.plan(
        occupancy,
        start=(3, 10),
        goal=(16, 10),
        safe_radius_m=0.1,
    )

    assert result.accepted
    assert result.reason == "accepted_goal_adjusted"
    assert result.path
    endpoint_cell = (
        int(math.floor(result.path[-1][0] / 0.1)),
        int(math.floor(result.path[-1][1] / 0.1)),
    )
    assert endpoint_cell == (13, 10)
    assert all(x < 1.4 for x, _ in result.path)


def test_planner_reachable_mask_uses_inflation_and_corner_rules() -> None:
    occupancy = np.zeros((9, 13), dtype=np.int8)
    occupancy[:, 6] = 100
    planner = R2EgoProxy2D(resolution_m=0.1)

    reachable = planner.reachable_mask(occupancy, start=(2, 4), safe_radius_m=0.1)

    assert reachable[4, 2]
    assert not np.any(reachable[:, 6:])

    blocked = planner.blocked_mask(occupancy, safe_radius_m=0.1)
    assert np.all(blocked[:, 5:8])


def test_vectorized_inflation_matches_euclidean_disk() -> None:
    occupancy = np.zeros((11, 13), dtype=np.int8)
    occupancy[2, 3] = 100
    occupancy[8, 10] = 100

    result = R2EgoProxy2D._inflate(occupancy, radius_cells=3)
    expected = np.zeros_like(result)
    for y, x in np.argwhere(occupancy >= 65):
        for target_y in range(occupancy.shape[0]):
            for target_x in range(occupancy.shape[1]):
                if (target_x - x) ** 2 + (target_y - y) ** 2 <= 9:
                    expected[target_y, target_x] = True

    assert np.array_equal(result, expected)


def test_planner_does_not_cut_diagonally_through_wall_corner() -> None:
    occupancy = np.zeros((3, 3), dtype=np.int8)
    occupancy[0, 1] = 100
    occupancy[1, 0] = 100
    result = R2EgoProxy2D(resolution_m=0.1).plan(
        occupancy,
        start=(0, 0),
        goal=(1, 1),
        safe_radius_m=0.0,
    )
    assert not result.accepted
    assert result.reason == "no_safe_path"


def test_replan_rate_limit_applies_even_when_every_attempt_fails() -> None:
    attempts = []
    last_attempt_s = -math.inf
    for now_s in np.arange(0.0, 2.01, 0.05):
        if rate_limit_due(float(now_s), last_attempt_s, rate_hz=2.0):
            attempts.append(float(now_s))
            last_attempt_s = float(now_s)
    assert attempts == pytest.approx([0.0, 0.5, 1.0, 1.5, 2.0])


def test_sentinel_fault_matrix_and_manual_no_reclaim() -> None:
    fsm = SentinelFSM()
    _critical_health(fsm)
    assert fsm.evaluate() == AutonomyMode.NORMAL

    fsm.set_health("camera", HealthSample(HealthLevel.FAIL, 2.0, 0.0, "unplugged"))
    assert fsm.evaluate() == AutonomyMode.NORMAL
    assert fsm.geometry_only

    fsm.set_health("camera", HealthSample(HealthLevel.OK, 0.0, 1.0, ""))
    fsm.set_health("npu", HealthSample(HealthLevel.FAIL, 0.1, 0.0, "stopped"))
    assert fsm.evaluate() == AutonomyMode.NORMAL
    assert fsm.geometry_only
    fsm.set_health("npu", HealthSample(HealthLevel.OK, 0.0, 1.0, ""))
    assert not fsm.geometry_only

    fsm.set_health("planner", HealthSample(HealthLevel.FAIL, 0.2, 0.0, "timeout"))
    assert fsm.evaluate() == AutonomyMode.BRAKE
    fsm.set_health("planner", HealthSample(HealthLevel.OK, 0.0, 1.0, ""))
    fsm.set_health("lidar", HealthSample(HealthLevel.FAIL, 1.5, 0.0, "stale"))
    assert fsm.evaluate() == AutonomyMode.HOVER
    fsm.set_health("lidar", HealthSample(HealthLevel.FAIL, 3.5, 0.0, "stale"))
    assert fsm.evaluate() == AutonomyMode.LAND

    fsm.set_manual_override(True)
    assert fsm.evaluate() == AutonomyMode.MANUAL
    _critical_health(fsm)
    assert fsm.evaluate() == AutonomyMode.MANUAL


def test_sentinel_f5_f7_f8_and_noncritical_link_semantics() -> None:
    fsm = SentinelFSM()
    _critical_health(fsm)

    fsm.set_health(
        "localization",
        HealthSample(HealthLevel.FAIL, 0.0, 0.01, "injected high covariance"),
    )
    assert fsm.evaluate() == AutonomyMode.RELOCALIZE

    fsm.set_health("localization", HealthSample(HealthLevel.OK, 0.0, 1.0, ""))
    fsm.set_health("ground_link", HealthSample(HealthLevel.FAIL, 0.0, 0.0, "drop"))
    assert fsm.evaluate() == AutonomyMode.NORMAL

    fsm.set_health("cpu", HealthSample(HealthLevel.FAIL, 0.0, 0.0, "load proxy"))
    assert fsm.evaluate() == AutonomyMode.NORMAL
    assert fsm.essential_only
    fsm.set_health("cpu", HealthSample(HealthLevel.OK, 0.0, 1.0, "recovered"))
    assert not fsm.essential_only

    fsm.set_battery(0.20)
    assert fsm.evaluate() == AutonomyMode.RETURN
    fsm.set_battery(0.10)
    assert fsm.evaluate() == AutonomyMode.LAND
    fsm.set_battery(1.0)
    assert fsm.evaluate() == AutonomyMode.NORMAL


def test_fault_schedule_covers_txt_f1_through_f8_without_overlap() -> None:
    schedule_path = Path(__file__).parents[1] / "config" / "fault_schedule.json"
    schedule = json.loads(schedule_path.read_text(encoding="utf-8"))
    events = schedule["events"]
    assert [event["at_s"] for event in events] == [3.0, 6.0, 9.0, 13.0, 17.0, 21.0, 28.0, 33.0]
    assert [event["fault_id"].split("_", 1)[0] for event in events] == [
        f"F{index}" for index in range(1, 9)
    ]
    assert [event["target_module"] for event in events] == [
        "camera",
        "npu",
        "planner",
        "lidar",
        "localization",
        "ground_link",
        "cpu",
        "battery",
    ]
    assert events[4]["action"] == "high_covariance"
    assert events[5]["severity"] == pytest.approx(0.20)
    assert events[6]["action"] == "load_proxy"
    assert events[-1]["at_s"] + events[-1]["duration_s"] <= 36.0


def test_fault_proxy_source_has_recovery_and_no_external_cpu_load_tool() -> None:
    package_dir = Path(__file__).parents[1] / "xq_autonomy"
    stack_source = (package_dir / "stack_node.py").read_text(encoding="utf-8")
    sentinel_source = (package_dir / "sentinel.py").read_text(encoding="utf-8")
    combined = stack_source + sentinel_source
    assert "_recover_fault_target" in stack_source
    assert "_expire_faults" in stack_source
    assert "_battery_fraction = 1.0" in stack_source
    assert "high_covariance" not in sentinel_source
    assert "ESSENTIAL_ONLY" in stack_source
    assert "essential_only" in sentinel_source
    assert "and not self.sentinel.essential_only" in stack_source
    assert "stress-ng" not in combined
    assert "subprocess" not in combined


def test_sentinel_stays_boot_until_first_finite_lidar_frame() -> None:
    fsm = SentinelFSM()
    assert fsm.evaluate() == AutonomyMode.BOOT
    for module in ("fcu", "localization", "planner"):
        fsm.set_health(module, HealthSample(HealthLevel.OK, 0.0, 1.0, ""))
    fsm.set_health(
        "lidar",
        HealthSample(HealthLevel.FAIL, math.inf, 0.0, "awaiting first point cloud"),
    )
    assert fsm.evaluate() == AutonomyMode.BOOT
    assert fsm.last_reason == "awaiting_first_sensor_frame"
    fsm.set_health("lidar", HealthSample(HealthLevel.OK, 0.0, 1.0, "first frame"))
    assert fsm.evaluate() == AutonomyMode.NORMAL


def test_sentinel_lidar_outage_boundaries() -> None:
    fsm = SentinelFSM()
    _critical_health(fsm)
    assert fsm.evaluate() == AutonomyMode.NORMAL
    for age_s in (0.75, 0.999):
        fsm.set_health("lidar", HealthSample(HealthLevel.FAIL, age_s, 0.2, "stale"))
        assert fsm.evaluate() == AutonomyMode.CAUTIOUS
    for age_s in (1.0, 2.999):
        fsm.set_health("lidar", HealthSample(HealthLevel.FAIL, age_s, 0.2, "stale"))
        assert fsm.evaluate() == AutonomyMode.HOVER
    fsm.set_health("lidar", HealthSample(HealthLevel.FAIL, 3.0, 0.2, "stale"))
    assert fsm.evaluate() == AutonomyMode.LAND


def test_packet_loss_is_scoped_reproducible_and_near_20_percent() -> None:
    def run() -> tuple[list[int], float]:
        relay = PacketLossRelay(drop_rate=0.20, delay_s=0.0, jitter_s=0.0, seed=42, max_age_s=10.0)
        delivered = []
        for seq in range(2000):
            now = seq * 0.01
            relay.send(Packet(seq, now, {"agent_id": "agent_02"}), now)
            delivered.extend(packet.seq for packet in relay.deliver(now))
        return delivered, relay.stats.drop_rate

    first, rate_a = run()
    second, rate_b = run()
    assert first == second
    assert rate_a == rate_b
    assert 0.17 <= rate_a <= 0.23


def test_ground_link_loss_is_zero_outside_fault_and_20_percent_inside() -> None:
    def run(healthy_prefix: int) -> tuple[list[int], dict]:
        link = GroundLinkHeartbeatRelay(
            seed=31415, delay_s=0.0, jitter_s=0.0, max_age_s=10.0
        )
        dropped = []
        for seq in range(healthy_prefix):
            now_s = seq * 0.01
            assert link.send(Packet(seq, now_s, {}), now_s)
            assert link.deliver(now_s)
        assert link.relay.stats.dropped == 0
        assert link.handle_fault(
            target_module="ground_link",
            action="drop",
            fault_id="F6_ground_link_20pct",
            severity=0.20,
            duration_s=20.0,
            now_s=10.0,
            seed=31415,
        )
        for offset in range(2000):
            seq = healthy_prefix + offset
            now_s = 10.0 + offset * 0.005
            if not link.send(Packet(seq, now_s, {}), now_s):
                dropped.append(offset)
            link.deliver(now_s)
        return dropped, link.stats_dict(19.999)

    dropped_a, stats_a = run(0)
    dropped_b, stats_b = run(137)
    assert dropped_a == dropped_b
    assert stats_a["fault_window_sent"] == 2000
    assert stats_a["fault_window_dropped"] == len(dropped_a)
    assert stats_a["fault_window_delivered"] == 2000 - len(dropped_a)
    assert stats_a["current_drop_rate"] == pytest.approx(0.20)
    assert 0.17 <= stats_a["observed_drop_rate"] <= 0.23
    assert stats_b["fault_window_dropped"] == stats_a["fault_window_dropped"]


def test_ground_link_ignores_other_faults_and_restores_after_window() -> None:
    link = GroundLinkHeartbeatRelay(
        seed=9, delay_s=0.0, jitter_s=0.0, max_age_s=10.0
    )
    assert not link.handle_fault(
        target_module="camera",
        action="drop",
        fault_id="not_network",
        severity=1.0,
        duration_s=10.0,
        now_s=0.0,
    )
    assert not link.active
    assert link.handle_fault(
        target_module="ground_link",
        action="drop",
        fault_id="window_01",
        severity=0.20,
        duration_s=1.0,
        now_s=5.0,
        seed=9,
    )
    for seq in range(100):
        now_s = 5.0 + seq * 0.009
        link.send(Packet(seq, now_s, {}), now_s)
        link.deliver(now_s)
    restored = link.stats_dict(6.0)
    assert not restored["active"]
    assert restored["fault_id"] == "window_01"
    assert restored["current_drop_rate"] == 0.0
    dropped_before_recovery = restored["dropped"]
    for seq in range(100, 300):
        now_s = 6.0 + seq * 0.01
        assert link.send(Packet(seq, now_s, {}), now_s)
        link.deliver(now_s)
    final = link.stats_dict(10.0)
    assert final["dropped"] == dropped_before_recovery
    assert final["fault_window_sent"] == 100
    assert final["sent"] == 300


def test_ground_link_clear_event_stops_loss_immediately() -> None:
    link = GroundLinkHeartbeatRelay(seed=3, delay_s=0.0, jitter_s=0.0)
    link.handle_fault(
        target_module="ground_link",
        action="drop",
        fault_id="window_clear",
        severity=1.0,
        duration_s=0.0,
        now_s=1.0,
        seed=3,
    )
    assert not link.send(Packet(0, 1.0, {}), 1.0)
    assert link.handle_fault(
        target_module="ground_link",
        action="recover",
        fault_id="window_clear",
        severity=0.0,
        duration_s=0.0,
        now_s=1.1,
    )
    assert link.send(Packet(1, 1.1, {}), 1.1)
    assert [packet.seq for packet in link.deliver(1.1)] == [1]
    stats = link.stats_dict(1.1)
    assert not stats["active"]
    assert stats["fault_window_sent"] == 1
    assert stats["fault_window_dropped"] == 1


def test_metric_formulas_and_official_gates() -> None:
    truth = np.column_stack([np.linspace(0.0, 5.0, 100), np.sin(np.linspace(0.0, 2.0, 100))])
    theta = 0.4
    rotation = np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])
    estimate = (rotation @ truth.T).T + np.array([3.0, -2.0])
    trajectory = trajectory_metrics(estimate, truth)
    assert trajectory["ate_rms_m"] < 1.0e-10

    frequency = frequency_metrics(np.arange(0.0, 3.01, 0.1))
    assert frequency["mean_hz"] == pytest.approx(10.0)
    assert frequency["worst_window_hz"] >= 10.0

    replans = replan_metrics([0.1] * 29 + [1.9], [True] * 30)
    assert replans["events"] == 30
    assert replans["official_2s_pass"]


def test_metrics_keep_transitions_and_compact_fault_summary() -> None:
    timeline = []
    assert append_state_transition(timeline, 1.0, "BOOT:awaiting_first_sensor_frame")
    assert not append_state_transition(timeline, 1.05, "BOOT:awaiting_first_sensor_frame")
    assert append_state_transition(timeline, 1.1, "NORMAL:all_critical_modules_healthy")
    assert len(timeline) == 2

    faults = [
        {"fault_id": "camera_01", "target_module": "camera"},
        {"fault_id": "planner_01", "target_module": "planner"},
        {"fault_id": "camera_01", "target_module": "camera"},
    ]
    summary = fault_evidence_summary(faults)
    assert summary["observed_count"] == 3
    assert summary["fault_ids_observed"] == ["camera_01", "planner_01"]
    assert summary["targets_observed"] == ["camera", "planner"]


def test_evaluation_truth_topic_is_absent_from_autonomy_stack_source() -> None:
    package_dir = Path(__file__).parents[1] / "xq_autonomy"
    stack_source = (package_dir / "stack_node.py").read_text(encoding="utf-8")
    metrics_source = (package_dir / "metrics_node.py").read_text(encoding="utf-8")
    assert "/xq/eval/" not in stack_source
    assert "/xq/eval/agent_01/ground_truth" in metrics_source


@pytest.mark.parametrize(
    "filename",
    [
        "stack_node.py",
        "metrics_node.py",
        "fault_injector_node.py",
        "network_relay_node.py",
    ],
)
def test_ros_entry_points_guard_external_shutdown(filename: str) -> None:
    source = (
        Path(__file__).parents[1] / "xq_autonomy" / filename
    ).read_text(encoding="utf-8")
    assert "ExternalShutdownException" in source
    assert "if rclpy.ok():" in source
    assert "rclpy.shutdown()" in source
