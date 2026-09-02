from pathlib import Path

from impact_fault_injection.fault_model import ActiveFaultSet, FaultSpec, load_schedule
from impact_fault_injection.supervisor import ResilientSupervisor, SafetyMode


def test_every_required_fault_is_in_matrix_schedule():
    path = Path(__file__).parents[1] / "config" / "p14_matrix_schedule.json"
    faults = load_schedule(path)
    assert {item.fault_type for item in faults} == {
        "lidar_dropout", "imu_dropout", "odom_delay", "planner_delay",
        "camera_failure", "20_percent_packet_loss", "cpu_load",
        "timestamp_jitter", "localization_covariance_inflation", "low_battery",
    }


def test_persistent_lidar_progresses_to_land():
    active = ActiveFaultSet()
    active.add(FaultSpec("f", "lidar_dropout", 0.0, 6.0, 1.0, 7), 10.0)
    supervisor = ResilientSupervisor()
    assert supervisor.evaluate(10.1, active).mode == SafetyMode.CAUTIOUS
    assert supervisor.evaluate(10.6, active).mode == SafetyMode.RECOVERY
    assert supervisor.evaluate(11.3, active).mode == SafetyMode.BRAKE
    assert supervisor.evaluate(12.0, active).mode == SafetyMode.HOVER
    assert supervisor.evaluate(13.1, active).mode == SafetyMode.LAND


def test_noncritical_failures_keep_local_mission_alive():
    supervisor = ResilientSupervisor()
    for fault_type in ("camera_failure", "20_percent_packet_loss"):
        active = ActiveFaultSet()
        active.add(FaultSpec("f", fault_type, 0.0, 2.0, 1.0, 9), 3.0)
        decision = supervisor.evaluate(3.1, active)
        assert decision.mode == SafetyMode.NORMAL
        assert decision.mission_continue


def test_manual_override_dominates_every_fault():
    active = ActiveFaultSet()
    active.add(FaultSpec("f", "lidar_dropout", 0.0, 10.0, 1.0, 1), 2.0)
    decision = ResilientSupervisor().evaluate(2.1, active, manual_override=True)
    assert decision.mode == SafetyMode.MANUAL
    assert not decision.mission_continue
