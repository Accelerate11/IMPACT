"""ROS-free temporal voxel surfel map used by P10 active perception."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class _VoxelMoments:
    count: int
    sum_xyz: np.ndarray
    sum_outer: np.ndarray
    observations: int
    last_frame: int
    last_seen_s: float


@dataclass(frozen=True)
class SurfelSnapshot:
    positions: np.ndarray
    normals: np.ndarray
    static_confidence: np.ndarray
    geometry_quality: np.ndarray
    last_seen_s: np.ndarray


class TemporalVoxelSurfelMap:
    """Accumulate registered scans into bounded, temporally confirmed surfels.

    The estimator only consumes map-frame point clouds.  A voxel's plane normal
    comes from the smallest eigenvector of its accumulated covariance.  Temporal
    recurrence and planar geometry remain separate fields so downstream logic
    cannot mistake a single clean scan for a confirmed static surface.
    """

    def __init__(
        self,
        *,
        voxel_size_m: float = 0.40,
        minimum_points: int = 8,
        confidence_observations: int = 4,
        minimum_geometry_quality: float = 0.10,
        stale_after_s: float = 30.0,
        maximum_voxels: int = 6000,
    ) -> None:
        values = (voxel_size_m, minimum_geometry_quality, stale_after_s)
        if not np.isfinite(values).all() or voxel_size_m <= 0.0 or stale_after_s <= 0.0:
            raise ValueError("invalid surfel map metric parameter")
        if minimum_points < 3 or confidence_observations < 1 or maximum_voxels < 1:
            raise ValueError("invalid surfel map count parameter")
        self.voxel_size_m = float(voxel_size_m)
        self.minimum_points = int(minimum_points)
        self.confidence_observations = int(confidence_observations)
        self.minimum_geometry_quality = float(minimum_geometry_quality)
        self.stale_after_s = float(stale_after_s)
        self.maximum_voxels = int(maximum_voxels)
        self._voxels: dict[tuple[int, int, int], _VoxelMoments] = {}
        self._frame = 0

    @staticmethod
    def _validate_points(points: np.ndarray) -> np.ndarray:
        array = np.asarray(points, dtype=np.float64)
        if array.ndim != 2 or array.shape[1] != 3:
            raise ValueError("surfel points must have shape (N,3)")
        if not np.isfinite(array).all():
            raise ValueError("surfel points must be finite")
        return array

    def update(self, points: np.ndarray, stamp_s: float) -> int:
        array = self._validate_points(points)
        if not np.isfinite(stamp_s) or stamp_s < 0.0:
            raise ValueError("invalid surfel timestamp")
        self._frame += 1
        if len(array) == 0:
            self.prune(stamp_s)
            return 0

        keys = np.floor(array / self.voxel_size_m).astype(np.int64)
        unique, inverse = np.unique(keys, axis=0, return_inverse=True)
        for group_index, key_array in enumerate(unique):
            group = array[inverse == group_index]
            key = tuple(int(value) for value in key_array)
            count = len(group)
            group_sum = np.sum(group, axis=0)
            group_outer = group.T @ group
            voxel = self._voxels.get(key)
            if voxel is None:
                self._voxels[key] = _VoxelMoments(
                    count=count,
                    sum_xyz=group_sum,
                    sum_outer=group_outer,
                    observations=1,
                    last_frame=self._frame,
                    last_seen_s=float(stamp_s),
                )
            else:
                voxel.count += count
                voxel.sum_xyz += group_sum
                voxel.sum_outer += group_outer
                if voxel.last_frame != self._frame:
                    voxel.observations += 1
                voxel.last_frame = self._frame
                voxel.last_seen_s = float(stamp_s)
        self.prune(stamp_s)
        return len(unique)

    def prune(self, now_s: float) -> None:
        stale = [
            key for key, voxel in self._voxels.items()
            if now_s - voxel.last_seen_s > self.stale_after_s
        ]
        for key in stale:
            del self._voxels[key]
        overflow = len(self._voxels) - self.maximum_voxels
        if overflow > 0:
            oldest = sorted(self._voxels, key=lambda key: self._voxels[key].last_seen_s)
            for key in oldest[:overflow]:
                del self._voxels[key]

    @staticmethod
    def _normal_and_quality(voxel: _VoxelMoments) -> tuple[np.ndarray, float] | None:
        if voxel.count < 3:
            return None
        centroid = voxel.sum_xyz / float(voxel.count)
        covariance = voxel.sum_outer / float(voxel.count) - np.outer(centroid, centroid)
        covariance = 0.5 * (covariance + covariance.T)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        eigenvalues = np.maximum(eigenvalues, 0.0)
        largest = float(eigenvalues[2])
        if largest <= 1.0e-12:
            return None
        # Standard planarity score: a line has lambda_1 ~= lambda_0 and is
        # rejected, while a plane has lambda_0 << lambda_1 <= lambda_2.
        quality = float(np.clip((eigenvalues[1] - eigenvalues[0]) / largest, 0.0, 1.0))
        normal = eigenvectors[:, 0]
        pivot = int(np.argmax(np.abs(normal)))
        if normal[pivot] < 0.0:
            normal = -normal
        return normal, quality

    def snapshot(self) -> SurfelSnapshot:
        records = []
        for key in sorted(self._voxels):
            voxel = self._voxels[key]
            if voxel.count < self.minimum_points:
                continue
            result = self._normal_and_quality(voxel)
            if result is None:
                continue
            normal, quality = result
            if quality < self.minimum_geometry_quality:
                continue
            centroid = voxel.sum_xyz / float(voxel.count)
            confidence = min(1.0, voxel.observations / float(self.confidence_observations))
            records.append((centroid, normal, confidence, quality, voxel.last_seen_s))
        if not records:
            empty3 = np.empty((0, 3), dtype=np.float64)
            empty1 = np.empty((0,), dtype=np.float64)
            return SurfelSnapshot(empty3, empty3.copy(), empty1, empty1.copy(), empty1.copy())
        return SurfelSnapshot(
            positions=np.stack([record[0] for record in records]),
            normals=np.stack([record[1] for record in records]),
            static_confidence=np.asarray([record[2] for record in records]),
            geometry_quality=np.asarray([record[3] for record in records]),
            last_seen_s=np.asarray([record[4] for record in records]),
        )

    @property
    def voxel_count(self) -> int:
        return len(self._voxels)
