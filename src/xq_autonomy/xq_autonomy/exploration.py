from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from .mapping import TDSemMap2D
from .types import Pose2D


@dataclass(frozen=True)
class CandidateScore:
    goal: Tuple[float, float]
    utility: float
    information: float
    observability: float
    travel_time: float
    risk: float
    energy: float
    repeat_cost: float


class OAER2D:
    """Observability-aware frontier selector used for SIL ablations."""

    def __init__(self, weights: Optional[Dict[str, float]] = None) -> None:
        self.weights = {
            "information": 1.0,
            "observability": 0.8,
            "travel_time": 0.35,
            "risk": 0.9,
            "energy": 0.20,
            "repeat": 0.45,
        }
        if weights:
            self.weights.update(weights)

    def score_candidates(
        self,
        mapping: TDSemMap2D,
        pose: Pose2D,
        weak_direction: Iterable[float],
        speed_mps: float = 0.8,
        reachable_mask: Optional[np.ndarray] = None,
    ) -> List[CandidateScore]:
        weak = np.asarray(list(weak_direction), dtype=float)
        norm = np.linalg.norm(weak)
        weak = weak / norm if norm > 1.0e-9 else np.array([1.0, 0.0])
        grid = mapping.occupancy_grid()
        if reachable_mask is not None:
            reachable_mask = np.asarray(reachable_mask, dtype=bool)
            if reachable_mask.shape != grid.shape:
                raise ValueError("reachable_mask must match the occupancy grid shape")
        clusters = mapping.frontier_clusters(min_cells=2)
        results: List[CandidateScore] = []
        for cluster in clusters:
            eligible = (
                cluster
                if reachable_mask is None
                else [cell for cell in cluster if reachable_mask[cell[1], cell[0]]]
            )
            if not eligible:
                continue
            # The arithmetic centroid of a closed frontier ring may lie back in
            # the already explored centre.  Keep the target on an actual
            # frontier cell, and when supplied keep it inside the planner's
            # start-connected safe component.
            mean_x = sum(c[0] for c in cluster) / len(cluster)
            mean_y = sum(c[1] for c in cluster) / len(cluster)
            gx, gy = min(
                eligible,
                key=lambda c: (c[0] - mean_x) ** 2 + (c[1] - mean_y) ** 2,
            )
            goal = mapping.grid_to_world(gx, gy)
            delta = np.array([goal[0] - pose.x, goal[1] - pose.y])
            distance = float(np.linalg.norm(delta))
            if distance < 0.15:
                continue
            direction = delta / distance
            observability = float(abs(np.dot(direction, weak)))
            information = float(min(1.0, len(cluster) / 40.0))
            travel_time = distance / max(speed_mps, 0.05)

            radius = max(2, int(round(0.5 / mapping.resolution_m)))
            y0, y1 = max(0, gy - radius), min(mapping.height, gy + radius + 1)
            x0, x1 = max(0, gx - radius), min(mapping.width, gx + radius + 1)
            patch = grid[y0:y1, x0:x1]
            occupied_ratio = float(np.count_nonzero(patch >= 100) / max(1, patch.size))
            unknown_ratio = float(np.count_nonzero(patch < 0) / max(1, patch.size))
            risk = float(np.clip(occupied_ratio * 3.0 + unknown_ratio * 0.25, 0.0, 1.0))
            energy = distance + max(0.0, travel_time - distance / 1.2) * 0.1
            repeat = 1.0 - unknown_ratio
            utility = (
                self.weights["information"] * information
                + self.weights["observability"] * observability
                - self.weights["travel_time"] * min(1.0, travel_time / 20.0)
                - self.weights["risk"] * risk
                - self.weights["energy"] * min(1.0, energy / 20.0)
                - self.weights["repeat"] * repeat
            )
            results.append(
                CandidateScore(
                    goal=goal,
                    utility=float(utility),
                    information=information,
                    observability=observability,
                    travel_time=travel_time,
                    risk=risk,
                    energy=energy,
                    repeat_cost=repeat,
                )
            )
        return sorted(results, key=lambda item: item.utility, reverse=True)

    def select_goal(
        self,
        mapping: TDSemMap2D,
        pose: Pose2D,
        weak_direction: Iterable[float],
        reachable_mask: Optional[np.ndarray] = None,
    ) -> Optional[CandidateScore]:
        scores = self.score_candidates(
            mapping,
            pose,
            weak_direction,
            reachable_mask=reachable_mask,
        )
        return scores[0] if scores else None
