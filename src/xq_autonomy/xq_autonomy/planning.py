from __future__ import annotations

import heapq
import math
import time
from collections import deque
from typing import Dict, Iterable, List, Optional, Tuple

import numpy as np

from .types import PlanResult


def rate_limit_due(now_s: float, last_run_s: float, rate_hz: float) -> bool:
    """Return whether a periodic action may run, including after a clock reset."""
    if rate_hz <= 0.0:
        raise ValueError("rate_hz must be positive")
    if not math.isfinite(last_run_s) or now_s < last_run_s:
        return True
    return now_s - last_run_s + 1.0e-12 >= 1.0 / rate_hz


def adaptive_safe_radius(
    body_radius_m: float,
    base_margin_m: float,
    covariance_xy: np.ndarray,
    speed_mps: float,
    latency_s: float,
    dynamic_margin_m: float = 0.0,
    k_sigma: float = 3.0,
) -> float:
    max_eigen = float(np.max(np.linalg.eigvalsh(np.asarray(covariance_xy, dtype=float))))
    return float(
        body_radius_m
        + base_margin_m
        + k_sigma * math.sqrt(max(max_eigen, 0.0))
        + max(0.0, speed_mps) * max(0.0, latency_s)
        + max(0.0, dynamic_margin_m)
    )


def brake_distance(speed_mps: float, deceleration_mps2: float, delay_s: float, margin_m: float) -> float:
    if deceleration_mps2 <= 0.0:
        raise ValueError("deceleration_mps2 must be positive")
    speed = max(0.0, speed_mps)
    return speed * speed / (2.0 * deceleration_mps2) + speed * max(0.0, delay_s) + margin_m


class R2EgoProxy2D:
    """Deadline-bounded inflated-grid A* proxy for R2-EGO integration tests."""

    def __init__(self, resolution_m: float, deadline_s: float = 0.8) -> None:
        self.resolution_m = float(resolution_m)
        self.deadline_s = float(deadline_s)

    @staticmethod
    def _inflate(occupancy: np.ndarray, radius_cells: int) -> np.ndarray:
        blocked = occupancy >= 65
        if radius_cells <= 0:
            return blocked
        result = np.zeros_like(blocked, dtype=bool)
        offsets = [
            (dx, dy)
            for dy in range(-radius_cells, radius_cells + 1)
            for dx in range(-radius_cells, radius_cells + 1)
            if dx * dx + dy * dy <= radius_cells * radius_cells
        ]
        height, width = blocked.shape
        for dx, dy in offsets:
            source_x0 = max(0, -dx)
            source_x1 = min(width, width - dx)
            source_y0 = max(0, -dy)
            source_y1 = min(height, height - dy)
            target_x0 = source_x0 + dx
            target_x1 = source_x1 + dx
            target_y0 = source_y0 + dy
            target_y1 = source_y1 + dy
            result[target_y0:target_y1, target_x0:target_x1] |= blocked[
                source_y0:source_y1,
                source_x0:source_x1,
            ]
        return result

    @staticmethod
    def _step_is_safe(
        blocked: np.ndarray,
        current: Tuple[int, int],
        nxt: Tuple[int, int],
    ) -> bool:
        """Reject occupied cells and diagonal moves that cut an obstacle corner."""
        height, width = blocked.shape
        nx, ny = nxt
        if not (0 <= nx < width and 0 <= ny < height) or blocked[ny, nx]:
            return False
        dx, dy = nx - current[0], ny - current[1]
        if dx != 0 and dy != 0:
            if blocked[current[1], current[0] + dx] or blocked[current[1] + dy, current[0]]:
                return False
        return True

    def blocked_mask(self, occupancy: np.ndarray, safe_radius_m: float) -> np.ndarray:
        """Build the inflated collision mask shared by planning and monitoring."""
        occupancy = np.asarray(occupancy)
        if occupancy.ndim != 2:
            raise ValueError("occupancy must be a 2-D grid")
        radius_cells = int(math.ceil(safe_radius_m / self.resolution_m))
        return self._inflate(occupancy, radius_cells)

    def reachable_mask(
        self,
        occupancy: np.ndarray,
        start: Tuple[int, int],
        safe_radius_m: float,
    ) -> np.ndarray:
        """Return the start-connected cells under the planner's safety model.

        Frontier utility alone cannot establish that a goal is navigable: an
        attractive frontier may lie across an inflated wall.  Exposing the
        exact start component lets the exploration layer reject such goals
        before A* while preserving the same inflation and corner-cut rules.
        """
        occupancy = np.asarray(occupancy)
        blocked = self.blocked_mask(occupancy, safe_radius_m)
        reachable = np.zeros_like(blocked, dtype=bool)
        height, width = blocked.shape
        if not (0 <= start[0] < width and 0 <= start[1] < height):
            return reachable
        if blocked[start[1], start[0]]:
            return reachable

        queue = deque([start])
        reachable[start[1], start[0]] = True
        neighbours = (
            (1, 0), (-1, 0), (0, 1), (0, -1),
            (1, 1), (1, -1), (-1, 1), (-1, -1),
        )
        while queue:
            current = queue.popleft()
            for dx, dy in neighbours:
                nxt = (current[0] + dx, current[1] + dy)
                nx, ny = nxt
                if not (0 <= nx < width and 0 <= ny < height):
                    continue
                if reachable[ny, nx] or not self._step_is_safe(blocked, current, nxt):
                    continue
                reachable[ny, nx] = True
                queue.append(nxt)
        return reachable

    @classmethod
    def _nearest_reachable_free_goal(
        cls,
        occupancy: np.ndarray,
        blocked: np.ndarray,
        start: Tuple[int, int],
        requested_goal: Tuple[int, int],
        deadline_at_s: float,
    ) -> Tuple[Optional[Tuple[int, int]], bool]:
        """Choose the closest known-free goal in the start's safe component.

        Merely snapping to the nearest free cell can choose a point across a wall.
        Flooding from ``start`` proves reachability under the same inflated-grid and
        no-corner-cutting rules used by A* before a replacement goal is accepted.
        """
        height, width = blocked.shape
        visited = np.zeros_like(blocked, dtype=bool)
        queue = deque([(start, 0)])
        visited[start[1], start[0]] = True
        best: Optional[Tuple[int, int]] = None
        best_key: Optional[Tuple[int, int, int, int]] = None
        expanded = 0
        neighbours = (
            (1, 0), (-1, 0), (0, 1), (0, -1),
            (1, 1), (1, -1), (-1, 1), (-1, -1),
        )
        while queue:
            current, depth = queue.popleft()
            expanded += 1
            if expanded % 256 == 0 and time.monotonic() > deadline_at_s:
                return None, True
            x, y = current
            if occupancy[y, x] == 0:
                distance_sq = (x - requested_goal[0]) ** 2 + (y - requested_goal[1]) ** 2
                key = (distance_sq, depth, y, x)
                if best_key is None or key < best_key:
                    best_key = key
                    best = current
            for dx, dy in neighbours:
                nxt = (x + dx, y + dy)
                nx, ny = nxt
                if not (0 <= nx < width and 0 <= ny < height) or visited[ny, nx]:
                    continue
                if not cls._step_is_safe(blocked, current, nxt):
                    continue
                visited[ny, nx] = True
                queue.append((nxt, depth + 1))
        return best, False

    def plan(
        self,
        occupancy: np.ndarray,
        start: Tuple[int, int],
        goal: Tuple[int, int],
        safe_radius_m: float,
        origin_xy: Tuple[float, float] = (0.0, 0.0),
        forced_delay_s: float = 0.0,
    ) -> PlanResult:
        started = time.monotonic()
        if forced_delay_s > 0.0:
            time.sleep(forced_delay_s)
        occupancy = np.asarray(occupancy)
        blocked = self.blocked_mask(occupancy, safe_radius_m)
        width, height = blocked.shape[1], blocked.shape[0]
        if not (0 <= start[0] < width and 0 <= start[1] < height):
            return PlanResult(False, reason="start_out_of_bounds", brake_fallback=True)
        if not (0 <= goal[0] < width and 0 <= goal[1] < height):
            return PlanResult(False, reason="goal_out_of_bounds", brake_fallback=True)
        if blocked[start[1], start[0]]:
            return PlanResult(False, reason="start_blocked", brake_fallback=True)

        goal_adjusted = False
        if blocked[goal[1], goal[0]] or occupancy[goal[1], goal[0]] != 0:
            goal, timed_out = self._nearest_reachable_free_goal(
                occupancy,
                blocked,
                start,
                goal,
                started + self.deadline_s,
            )
            if timed_out:
                return PlanResult(
                    False,
                    latency_s=time.monotonic() - started,
                    brake_fallback=True,
                    reason="deadline_exceeded",
                    safe_radius_m=safe_radius_m,
                )
            if goal is None:
                return PlanResult(
                    False,
                    latency_s=time.monotonic() - started,
                    brake_fallback=True,
                    reason="no_reachable_free_goal",
                    safe_radius_m=safe_radius_m,
                )
            goal_adjusted = True

        frontier: List[Tuple[float, Tuple[int, int]]] = [(0.0, start)]
        came_from: Dict[Tuple[int, int], Optional[Tuple[int, int]]] = {start: None}
        cost: Dict[Tuple[int, int], float] = {start: 0.0}
        neighbours = (
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0)),
            (-1, 1, math.sqrt(2.0)), (-1, -1, math.sqrt(2.0)),
        )
        reached = False
        while frontier:
            if time.monotonic() - started > self.deadline_s:
                return PlanResult(
                    False,
                    latency_s=time.monotonic() - started,
                    brake_fallback=True,
                    reason="deadline_exceeded",
                    safe_radius_m=safe_radius_m,
                )
            _, current = heapq.heappop(frontier)
            if current == goal:
                reached = True
                break
            for dx, dy, step in neighbours:
                nxt = (current[0] + dx, current[1] + dy)
                if not (0 <= nxt[0] < width and 0 <= nxt[1] < height):
                    continue
                if not self._step_is_safe(blocked, current, nxt):
                    continue
                unknown_penalty = 1.8 if occupancy[nxt[1], nxt[0]] < 0 else 1.0
                new_cost = cost[current] + step * unknown_penalty
                if nxt not in cost or new_cost < cost[nxt]:
                    cost[nxt] = new_cost
                    heuristic = math.hypot(goal[0] - nxt[0], goal[1] - nxt[1])
                    heapq.heappush(frontier, (new_cost + heuristic, nxt))
                    came_from[nxt] = current

        latency = time.monotonic() - started
        if not reached:
            return PlanResult(
                False,
                latency_s=latency,
                brake_fallback=True,
                reason="no_safe_path",
                safe_radius_m=safe_radius_m,
            )
        cells = []
        current: Optional[Tuple[int, int]] = goal
        while current is not None:
            cells.append(current)
            current = came_from[current]
        cells.reverse()
        path = [
            (
                origin_xy[0] + (x + 0.5) * self.resolution_m,
                origin_xy[1] + (y + 0.5) * self.resolution_m,
            )
            for x, y in cells
        ]
        return PlanResult(
            True,
            path=path,
            latency_s=latency,
            brake_fallback=False,
            reason="accepted_goal_adjusted" if goal_adjusted else "accepted",
            safe_radius_m=safe_radius_m,
        )
