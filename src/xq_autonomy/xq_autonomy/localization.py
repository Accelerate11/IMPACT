from __future__ import annotations

import math
from typing import Iterable, Optional

import numpy as np

from .geometry import wrap_angle
from .types import LocalizationQualityData, Pose2D


class DafLioProxy2D:
    """Deterministic 2-D proxy for DAF-LIO interface and feedback testing.

    The class integrates commanded planar motion and derives a directional
    observability score from LiDAR hit directions.  It intentionally does not
    claim to implement FAST-LIO2's ESIKF or point-to-plane optimizer.
    """

    def __init__(
        self,
        seed: int = 20260820,
        linear_noise_mps: float = 0.004,
        yaw_noise_rps: float = 0.002,
    ) -> None:
        self.pose = Pose2D()
        self._rng = np.random.default_rng(seed)
        self._linear_noise = float(linear_noise_mps)
        self._yaw_noise = float(yaw_noise_rps)
        self._covariance = np.eye(2, dtype=float) * 1.0e-4
        self._quality = LocalizationQualityData(
            covariance_xy=self._covariance.copy(),
            weak_direction=np.array([1.0, 0.0]),
            eigenvalues=np.array([1.0, 1.0]),
            degeneracy_score=0.0,
            innovation_rms=0.0,
            effective_points=0,
            map_match_score=0.0,
        )

    @property
    def quality(self) -> LocalizationQualityData:
        return self._quality

    def observe(self, points_xy: Iterable[Iterable[float]]) -> LocalizationQualityData:
        points = np.asarray(list(points_xy), dtype=float)
        if points.ndim != 2 or points.shape[1] != 2:
            raise ValueError("points_xy must have shape (N, 2)")
        finite = np.isfinite(points).all(axis=1)
        points = points[finite]
        ranges = np.linalg.norm(points, axis=1)
        points = points[ranges > 0.15]
        ranges = ranges[ranges > 0.15]
        count = int(points.shape[0])
        if count < 8:
            eig = np.array([1.0e-6, 1.0e-6])
            weak = np.array([1.0, 0.0])
            degeneracy = 1.0
            match = 0.0
            innovation = 1.0
        else:
            directions = points / ranges[:, None]
            # Directional excitation matrix.  An evenly distributed scan has
            # similar eigenvalues; a corridor / single plane has one weak axis.
            information = directions.T @ directions / float(count)
            eig, vectors = np.linalg.eigh(information)
            order = np.argsort(eig)
            eig = eig[order]
            vectors = vectors[:, order]
            weak = vectors[:, 0]
            ratio = float(eig[0] / max(eig[-1], 1.0e-12))
            degeneracy = float(np.clip(1.0 - ratio, 0.0, 1.0))
            angular_balance = float(np.clip(2.0 * eig[0], 0.0, 1.0))
            density = float(np.clip(count / 720.0, 0.0, 1.0))
            match = angular_balance * density
            innovation = float(np.clip(np.std(ranges) / max(np.mean(ranges), 1.0e-6), 0.0, 1.0))

        weak_outer = np.outer(weak, weak)
        target_cov = np.eye(2) * 2.0e-4 + weak_outer * (degeneracy ** 2) * 0.04
        self._covariance = 0.8 * self._covariance + 0.2 * target_cov
        self._quality = LocalizationQualityData(
            covariance_xy=self._covariance.copy(),
            weak_direction=weak.copy(),
            eigenvalues=eig.copy(),
            degeneracy_score=degeneracy,
            innovation_rms=innovation,
            effective_points=count,
            map_match_score=match,
        )
        return self._quality

    def step(self, linear_mps: float, yaw_rate_rps: float, dt_s: float) -> Pose2D:
        if dt_s <= 0.0:
            return self.pose
        q = self._quality
        weak = np.asarray(q.weak_direction, dtype=float)
        directional_sigma = self._linear_noise * (1.0 + 10.0 * q.degeneracy_score)
        isotropic = self._rng.normal(0.0, self._linear_noise * math.sqrt(dt_s), size=2)
        directional = weak * self._rng.normal(0.0, directional_sigma * math.sqrt(dt_s))
        yaw_noise = self._rng.normal(
            0.0,
            self._yaw_noise * (1.0 + 3.0 * q.degeneracy_score) * math.sqrt(dt_s),
        )
        self.pose.yaw = wrap_angle(self.pose.yaw + yaw_rate_rps * dt_s + yaw_noise)
        forward = np.array([math.cos(self.pose.yaw), math.sin(self.pose.yaw)])
        delta = forward * linear_mps * dt_s + isotropic + directional
        self.pose.x += float(delta[0])
        self.pose.y += float(delta[1])
        process = np.eye(2) * (self._linear_noise ** 2) * dt_s
        process += np.outer(weak, weak) * (directional_sigma ** 2) * dt_s
        self._covariance += process
        return self.pose

    def reset(self, pose: Optional[Pose2D] = None) -> None:
        self.pose = pose if pose is not None else Pose2D()
        self._covariance = np.eye(2, dtype=float) * 1.0e-4

