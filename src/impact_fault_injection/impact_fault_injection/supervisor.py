"""P14 safety state machine derived from sensor/control invariants."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from .fault_model import ActiveFaultSet


class SafetyMode(str, Enum):
    NORMAL = "NORMAL"
    CAUTIOUS = "CAUTIOUS"
    RECOVERY = "RECOVERY"
    BRAKE = "BRAKE"
    HOVER = "HOVER"
    RETURN = "RETURN"
    LAND = "LAND"
    MANUAL = "MANUAL"


@dataclass(frozen=True)
class SafetyDecision:
    mode: SafetyMode
    reason: str
    geometry_only: bool = False
    essential_only: bool = False
    mission_continue: bool = True


class ResilientSupervisor:
    """Priority-ordered response; safety failures dominate mission utility."""

    def evaluate(
        self,
        now_s: float,
        faults: ActiveFaultSet,
        *,
        manual_override: bool = False,
    ) -> SafetyDecision:
        if manual_override:
            return SafetyDecision(SafetyMode.MANUAL, "manual_override", mission_continue=False)

        lidar_age = faults.age_s(now_s, "lidar_dropout")
        if lidar_age is not None:
            if lidar_age >= 3.0:
                return SafetyDecision(SafetyMode.LAND, "lidar_persistent_3s", mission_continue=False)
            if lidar_age >= 1.8:
                return SafetyDecision(SafetyMode.HOVER, "lidar_persistent_1p8s", mission_continue=False)
            if lidar_age >= 1.2:
                return SafetyDecision(SafetyMode.BRAKE, "lidar_persistent_1p2s", mission_continue=False)
            if lidar_age >= 0.5:
                return SafetyDecision(SafetyMode.RECOVERY, "lidar_imu_propagation_limit")
            return SafetyDecision(SafetyMode.CAUTIOUS, "lidar_short_dropout")

        battery = faults.severity(now_s, "low_battery", default=1.0)
        if faults.has(now_s, "low_battery"):
            if battery <= 0.15:
                return SafetyDecision(SafetyMode.LAND, "critical_battery", mission_continue=False)
            return SafetyDecision(SafetyMode.RETURN, "low_battery", mission_continue=False)

        if faults.has(now_s, "planner_delay"):
            return SafetyDecision(SafetyMode.BRAKE, "planner_timeout", mission_continue=False)

        if faults.has(now_s, "odom_delay"):
            return SafetyDecision(SafetyMode.RECOVERY, "odometry_stale")
        if faults.has(now_s, "localization_covariance_inflation"):
            return SafetyDecision(SafetyMode.RECOVERY, "localization_covariance_inflated")

        geometry_only = faults.has(now_s, "camera_failure")
        essential_only = faults.has(now_s, "cpu_load")
        if faults.has(now_s, "imu_dropout"):
            return SafetyDecision(SafetyMode.CAUTIOUS, "imu_dropout_lidar_only", geometry_only, essential_only)
        if faults.has(now_s, "timestamp_jitter"):
            return SafetyDecision(SafetyMode.CAUTIOUS, "timestamp_jitter", geometry_only, essential_only)
        if essential_only:
            return SafetyDecision(SafetyMode.CAUTIOUS, "cpu_load_shed_noncritical", geometry_only, True)

        # Camera and ground-link loss do not remove the LiDAR/IMU local flight loop.
        if geometry_only:
            return SafetyDecision(SafetyMode.NORMAL, "camera_failed_geometry_navigation", True)
        if faults.has(now_s, "20_percent_packet_loss"):
            return SafetyDecision(SafetyMode.NORMAL, "ground_link_loss_local_mission_continues")
        return SafetyDecision(SafetyMode.NORMAL, "all_critical_channels_healthy")
