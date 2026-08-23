from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
from typing import Any, Dict, List, Tuple


class HealthLevel(IntEnum):
    OK = 0
    WARN = 1
    FAIL = 2


class AutonomyMode(IntEnum):
    BOOT = 0
    SELF_CHECK = 1
    NORMAL = 2
    CAUTIOUS = 3
    RELOCALIZE = 4
    BRAKE = 5
    HOVER = 6
    RETURN = 7
    LAND = 8
    MANUAL = 9
    ABORT = 10


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass
class LocalizationQualityData:
    covariance_xy: Any
    weak_direction: Any
    eigenvalues: Any
    degeneracy_score: float
    innovation_rms: float
    effective_points: int
    map_match_score: float


@dataclass
class HealthSample:
    level: HealthLevel = HealthLevel.OK
    age_s: float = 0.0
    quality: float = 1.0
    reason: str = ""


@dataclass
class PlanResult:
    accepted: bool
    path: List[Tuple[float, float]] = field(default_factory=list)
    latency_s: float = 0.0
    brake_fallback: bool = False
    reason: str = ""
    safe_radius_m: float = 0.0


@dataclass
class Packet:
    seq: int
    stamp_s: float
    payload: Dict[str, Any]

