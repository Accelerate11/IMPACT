from __future__ import annotations

import math
from collections import deque
from typing import Iterable, List, Sequence, Tuple

import numpy as np

from .geometry import bresenham
from .types import Pose2D


class TDSemMap2D:
    """Bounded 2-D time-decaying occupancy map used by the SIL stack."""

    def __init__(
        self,
        width_m: float = 24.0,
        height_m: float = 24.0,
        resolution_m: float = 0.10,
        dynamic_ttl_s: float = 2.5,
    ) -> None:
        if resolution_m <= 0.0:
            raise ValueError("resolution_m must be positive")
        self.resolution_m = float(resolution_m)
        self.width = int(math.ceil(width_m / resolution_m))
        self.height = int(math.ceil(height_m / resolution_m))
        self.origin_x = -0.5 * self.width * self.resolution_m
        self.origin_y = -0.5 * self.height * self.resolution_m
        self.dynamic_ttl_s = float(dynamic_ttl_s)
        shape = (self.height, self.width)
        self.log_odds = np.zeros(shape, dtype=np.float32)
        self.observed = np.zeros(shape, dtype=bool)
        self.dynamic_confidence = np.zeros(shape, dtype=np.float32)
        self.last_seen_s = np.full(shape, -np.inf, dtype=np.float64)
        self.last_free_s = np.full(shape, -np.inf, dtype=np.float64)

    def in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.width and 0 <= gy < self.height

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        gx = int(math.floor((x - self.origin_x) / self.resolution_m))
        gy = int(math.floor((y - self.origin_y) / self.resolution_m))
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        return (
            self.origin_x + (gx + 0.5) * self.resolution_m,
            self.origin_y + (gy + 0.5) * self.resolution_m,
        )

    def decay(self, now_s: float) -> None:
        age = np.maximum(0.0, now_s - self.last_seen_s)
        valid = np.isfinite(age) & (self.dynamic_confidence > 0.0)
        if np.any(valid):
            self.dynamic_confidence[valid] *= np.exp(
                -age[valid] / max(self.dynamic_ttl_s, 1.0e-3)
            ).astype(np.float32)
            stale = valid & (age > self.dynamic_ttl_s) & (self.dynamic_confidence < 0.15)
            # A transient obstacle that disappeared should not remain a wall.
            self.log_odds[stale] = np.minimum(self.log_odds[stale], -0.6)
            self.dynamic_confidence[stale] = 0.0

    def update(
        self,
        pose: Pose2D,
        points_xy_local: Iterable[Iterable[float]],
        now_s: float,
        max_rays: int = 360,
    ) -> int:
        self.decay(now_s)
        points = np.asarray(list(points_xy_local), dtype=float)
        if points.size == 0:
            return 0
        points = points.reshape((-1, 2))
        points = points[np.isfinite(points).all(axis=1)]
        if points.shape[0] > max_rays:
            indices = np.linspace(0, points.shape[0] - 1, max_rays, dtype=int)
            points = points[indices]

        cg, sg = math.cos(pose.yaw), math.sin(pose.yaw)
        rotation = np.array([[cg, -sg], [sg, cg]])
        world_points = (rotation @ points.T).T + np.array([pose.x, pose.y])
        ox, oy = self.world_to_grid(pose.x, pose.y)
        if not self.in_bounds(ox, oy):
            return 0

        hits = 0
        for wx, wy in world_points:
            gx, gy = self.world_to_grid(float(wx), float(wy))
            if not self.in_bounds(gx, gy):
                continue
            cells = bresenham(ox, oy, gx, gy)
            for fx, fy in cells[:-1]:
                if not self.in_bounds(fx, fy):
                    continue
                was_occupied = self.log_odds[fy, fx] > 0.8
                self.log_odds[fy, fx] = max(-4.0, float(self.log_odds[fy, fx]) - 0.35)
                self.observed[fy, fx] = True
                self.last_free_s[fy, fx] = now_s
                if was_occupied:
                    self.dynamic_confidence[fy, fx] = min(
                        1.0, float(self.dynamic_confidence[fy, fx]) + 0.25
                    )
            recently_free = now_s - self.last_free_s[gy, gx] < 1.0
            self.log_odds[gy, gx] = min(4.0, float(self.log_odds[gy, gx]) + 0.85)
            self.observed[gy, gx] = True
            self.last_seen_s[gy, gx] = now_s
            if recently_free:
                self.dynamic_confidence[gy, gx] = min(
                    1.0, float(self.dynamic_confidence[gy, gx]) + 0.45
                )
            hits += 1
        return hits

    def occupancy_grid(self) -> np.ndarray:
        grid = np.full(self.log_odds.shape, -1, dtype=np.int8)
        grid[self.observed & (self.log_odds < -0.25)] = 0
        grid[self.observed & (self.log_odds >= -0.25) & (self.log_odds <= 0.55)] = 50
        grid[self.observed & (self.log_odds > 0.55)] = 100
        # Dynamic occupied cells remain obstacles until TTL decay clears them.
        grid[self.dynamic_confidence > 0.25] = 100
        return grid

    def known_fraction(self) -> float:
        return float(np.count_nonzero(self.observed) / self.observed.size)

    def frontier_cells(self) -> List[Tuple[int, int]]:
        free = self.observed & (self.log_odds < -0.25)
        unknown = ~self.observed
        frontier: List[Tuple[int, int]] = []
        for gy, gx in np.argwhere(free):
            for nx, ny in ((gx + 1, gy), (gx - 1, gy), (gx, gy + 1), (gx, gy - 1)):
                if self.in_bounds(nx, ny) and unknown[ny, nx]:
                    frontier.append((gx, gy))
                    break
        return frontier

    def frontier_clusters(self, min_cells: int = 3) -> List[List[Tuple[int, int]]]:
        remaining = set(self.frontier_cells())
        clusters: List[List[Tuple[int, int]]] = []
        while remaining:
            seed = remaining.pop()
            queue = deque([seed])
            cluster = [seed]
            while queue:
                x, y = queue.popleft()
                for n in (
                    (x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1),
                    (x + 1, y + 1), (x - 1, y - 1), (x + 1, y - 1), (x - 1, y + 1),
                ):
                    if n in remaining:
                        remaining.remove(n)
                        queue.append(n)
                        cluster.append(n)
            if len(cluster) >= min_cells:
                clusters.append(cluster)
        return clusters

