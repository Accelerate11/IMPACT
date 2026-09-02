"""Pure P12 dynamic-path collision and passage reopening logic."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


def path_obstruction(
    dynamic_points: np.ndarray,
    start_xyz: np.ndarray,
    goal_xyz: np.ndarray,
    *,
    clearance_radius_m: float,
    lookahead_m: float,
) -> tuple[bool, float]:
    points = np.asarray(dynamic_points, dtype=np.float64)
    start = np.asarray(start_xyz, dtype=np.float64).reshape(3)
    goal = np.asarray(goal_xyz, dtype=np.float64).reshape(3)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("dynamic_points must have shape (N,3)")
    if not np.isfinite((start, goal)).all() or clearance_radius_m <= 0.0 or lookahead_m <= 0.0:
        raise ValueError("invalid path obstruction parameter")
    if len(points) == 0:
        return False, math.inf
    direction = goal - start
    length = float(np.linalg.norm(direction))
    if length <= 1.0e-9:
        return False, math.inf
    unit = direction / length
    relative = points - start
    along = relative @ unit
    lateral = np.linalg.norm(relative - np.outer(along, unit), axis=1)
    mask = (
        (along >= 0.20)
        & (along <= min(length, lookahead_m))
        & (lateral <= clearance_radius_m)
    )
    if not np.any(mask):
        return False, math.inf
    return True, float(np.min(along[mask]))


@dataclass(frozen=True)
class PassageDecision:
    state: str
    brake: bool
    obstacle_confirmed: bool
    passage_reopened: bool


class DynamicPassageGate:
    """Fail-closed state machine with debounced reopening after TTL clearance."""

    def __init__(self, clear_confirmation_s: float = 1.0) -> None:
        if not math.isfinite(clear_confirmation_s) or clear_confirmation_s < 0.0:
            raise ValueError("invalid clear confirmation time")
        self.clear_confirmation_s = float(clear_confirmation_s)
        self._blocked_once = False
        self._clear_since_s: float | None = None
        self._reopened = False

    def update(self, blocked: bool, now_s: float) -> PassageDecision:
        if not math.isfinite(now_s) or now_s < 0.0:
            raise ValueError("invalid passage timestamp")
        if blocked:
            self._blocked_once = True
            self._clear_since_s = None
            self._reopened = False
            return PassageDecision("BRAKE_DYNAMIC", True, True, False)
        if not self._blocked_once:
            return PassageDecision("CLEAR_INITIAL", False, False, False)
        if self._clear_since_s is None:
            self._clear_since_s = float(now_s)
        if now_s - self._clear_since_s < self.clear_confirmation_s:
            return PassageDecision("VERIFY_TTL_CLEAR", True, True, False)
        first_reopen = not self._reopened
        self._reopened = True
        return PassageDecision("PASSAGE_REOPENED", False, True, first_reopen)

