from __future__ import annotations

import math
from typing import Dict

from .types import AutonomyMode, HealthLevel, HealthSample


class SentinelFSM:
    """Deterministic safety supervisor matching the TXT fault semantics."""

    def __init__(self) -> None:
        self.mode = AutonomyMode.BOOT
        self.health: Dict[str, HealthSample] = {}
        self.manual_override = False
        self.battery_fraction = 1.0
        self.last_reason = "boot"

    def set_health(self, module: str, sample: HealthSample) -> None:
        self.health[module] = sample

    def set_manual_override(self, active: bool) -> None:
        self.manual_override = bool(active)

    def set_battery(self, fraction: float) -> None:
        self.battery_fraction = max(0.0, min(1.0, float(fraction)))

    def _get(self, module: str) -> HealthSample:
        return self.health.get(module, HealthSample(HealthLevel.FAIL, 1.0e9, 0.0, "missing"))

    def evaluate(self) -> AutonomyMode:
        if self.manual_override:
            self.mode = AutonomyMode.MANUAL
            self.last_reason = "pilot_override"
            return self.mode

        fcu = self._get("fcu")
        localization = self._get("localization")
        lidar = self._get("lidar")
        planner = self._get("planner")

        critical = ("fcu", "localization", "lidar", "planner")
        if self.mode == AutonomyMode.BOOT and (
            any(module not in self.health for module in critical)
            or not math.isfinite(lidar.age_s)
        ):
            self.last_reason = "awaiting_first_sensor_frame"
            return self.mode

        if fcu.level == HealthLevel.FAIL:
            self.mode = AutonomyMode.LAND
            self.last_reason = "fcu_link_fail"
        elif self.battery_fraction <= 0.15:
            self.mode = AutonomyMode.LAND
            self.last_reason = "critical_battery"
        elif self.battery_fraction <= 0.25:
            self.mode = AutonomyMode.RETURN
            self.last_reason = "low_battery"
        elif lidar.level == HealthLevel.FAIL:
            if lidar.age_s >= 3.0:
                self.mode = AutonomyMode.LAND
                self.last_reason = "lidar_long_outage"
            elif lidar.age_s >= 1.0:
                self.mode = AutonomyMode.HOVER
                self.last_reason = "lidar_outage"
            else:
                self.mode = AutonomyMode.CAUTIOUS
                self.last_reason = "lidar_short_outage_imu_propagation"
        elif planner.level == HealthLevel.FAIL:
            self.mode = AutonomyMode.BRAKE
            self.last_reason = "planner_fail_or_timeout"
        elif localization.level == HealthLevel.FAIL or localization.quality < 0.2:
            self.mode = AutonomyMode.RELOCALIZE
            self.last_reason = "localization_unreliable"
        elif lidar.level == HealthLevel.WARN or localization.level == HealthLevel.WARN:
            self.mode = AutonomyMode.CAUTIOUS
            self.last_reason = "degraded_geometry"
        else:
            # Camera, NPU, CPU-load proxy and ground link are deliberately not
            # safety-critical. Their degraded-service flags are exposed by the
            # properties below without interrupting the essential flight loop.
            self.mode = AutonomyMode.NORMAL
            self.last_reason = "all_critical_modules_healthy"
        return self.mode

    @property
    def geometry_only(self) -> bool:
        camera = self.health.get("camera")
        npu = self.health.get("npu")
        return any(sample is not None and sample.level == HealthLevel.FAIL for sample in (camera, npu))

    @property
    def essential_only(self) -> bool:
        """Whether non-critical services are shed by the project CPU proxy."""
        cpu = self.health.get("cpu")
        return cpu is not None and cpu.level == HealthLevel.FAIL
