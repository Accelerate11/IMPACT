"""Pure deterministic fault schedule and active-window primitives."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import random
from typing import Iterable


SUPPORTED_FAULTS = frozenset(
    {
        "lidar_dropout",
        "imu_dropout",
        "odom_delay",
        "planner_delay",
        "camera_failure",
        "20_percent_packet_loss",
        "cpu_load",
        "timestamp_jitter",
        "localization_covariance_inflation",
        "low_battery",
    }
)


@dataclass(frozen=True)
class FaultSpec:
    fault_id: str
    fault_type: str
    start_time_s: float
    duration_s: float
    severity: float
    seed: int

    @property
    def end_time_s(self) -> float:
        return self.start_time_s + self.duration_s

    def active(self, elapsed_s: float) -> bool:
        return self.start_time_s <= elapsed_s < self.end_time_s


def load_schedule(path: str | Path) -> list[FaultSpec]:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    default_seed = int(payload.get("seed", 20260828))
    output: list[FaultSpec] = []
    seen: set[str] = set()
    for raw in payload.get("faults", []):
        fault_type = str(raw["type"])
        if fault_type not in SUPPORTED_FAULTS:
            raise ValueError(f"unsupported fault type: {fault_type}")
        fault_id = str(raw.get("id", fault_type))
        if fault_id in seen:
            raise ValueError(f"duplicate fault id: {fault_id}")
        seen.add(fault_id)
        item = FaultSpec(
            fault_id=fault_id,
            fault_type=fault_type,
            start_time_s=float(raw["start_time_s"]),
            duration_s=float(raw["duration_s"]),
            severity=float(raw.get("severity", 1.0)),
            seed=int(raw.get("seed", default_seed)),
        )
        if item.start_time_s < 0.0 or item.duration_s <= 0.0:
            raise ValueError(f"invalid fault window: {fault_id}")
        output.append(item)
    return sorted(output, key=lambda item: (item.start_time_s, item.fault_id))


class ActiveFaultSet:
    """Fault windows keyed by ID, with deterministic sampling per event."""

    def __init__(self) -> None:
        self._faults: dict[str, tuple[FaultSpec, float]] = {}
        self._rng: dict[str, random.Random] = {}

    def add(self, spec: FaultSpec, observed_start_s: float) -> None:
        self._faults[spec.fault_id] = (spec, float(observed_start_s))
        self._rng[spec.fault_id] = random.Random(spec.seed)

    def purge(self, now_s: float) -> None:
        expired = [
            fault_id
            for fault_id, (spec, start_s) in self._faults.items()
            if now_s >= start_s + spec.duration_s
        ]
        for fault_id in expired:
            self._faults.pop(fault_id, None)
            self._rng.pop(fault_id, None)

    def active(self, now_s: float, fault_type: str | None = None) -> list[tuple[FaultSpec, float]]:
        self.purge(now_s)
        values = list(self._faults.values())
        if fault_type is not None:
            values = [item for item in values if item[0].fault_type == fault_type]
        return sorted(values, key=lambda item: item[0].fault_id)

    def has(self, now_s: float, fault_type: str) -> bool:
        return bool(self.active(now_s, fault_type))

    def age_s(self, now_s: float, fault_type: str) -> float | None:
        values = self.active(now_s, fault_type)
        return max((now_s - start_s for _, start_s in values), default=None)

    def severity(self, now_s: float, fault_type: str, default: float = 0.0) -> float:
        values = self.active(now_s, fault_type)
        return max((spec.severity for spec, _ in values), default=default)

    def ids(self, now_s: float) -> list[str]:
        return [spec.fault_id for spec, _ in self.active(now_s)]

    def random(self, fault_id: str) -> float:
        return self._rng[fault_id].random()


def schedule_types(specs: Iterable[FaultSpec]) -> set[str]:
    return {item.fault_type for item in specs}
