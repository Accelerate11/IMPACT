"""ROS-free P12 temporal dynamic voxel map.

The map deliberately keeps static and dynamic evidence in separate fields.
Only dynamic confidence decays; confirmed static structure is never removed by
the TTL path.  This makes the safety property explicit instead of relying on a
single occupancy probability with ambiguous forgetting semantics.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import time

import numpy as np


@dataclass
class DynamicVoxel:
    occupancy_logodds: float = 0.0
    hit_count: int = 0
    free_count: int = 0
    static_confidence: float = 0.0
    dynamic_confidence: float = 0.0
    dynamic_observations: int = 0
    dynamic_first_hit_s: float = -math.inf
    last_dynamic_hit_s: float = -math.inf
    last_seen_s: float = -math.inf
    last_free_s: float = -math.inf
    last_decay_s: float = -math.inf
    path_promoted: bool = False
    dynamic_history: bool = False
    baseline_static: bool = False
    reversible_static: bool = False
    consecutive_free_count: int = 0


class TemporalDynamicVoxelMap:
    """Sparse 3-D LiDAR map with geometry-only dynamic detection and TTL."""

    def __init__(
        self,
        *,
        voxel_size_m: float = 0.25,
        dynamic_ttl_s: float = 3.0,
        static_confirmation_hits: int = 6,
        free_confirmation_rays: int = 3,
        dynamic_occupied_threshold: float = 0.35,
        dynamic_clear_threshold: float = 0.08,
        dynamic_confirmation_hits: int = 3,
        post_dynamic_static_confirmation_s: float = 0.0,
        reversible_static_ttl_s: float = 0.0,
        maximum_voxels: int = 120000,
    ) -> None:
        finite = (
            voxel_size_m,
            dynamic_ttl_s,
            dynamic_occupied_threshold,
            dynamic_clear_threshold,
            post_dynamic_static_confirmation_s,
            reversible_static_ttl_s,
        )
        if (
            not np.isfinite(finite).all()
            or voxel_size_m <= 0.0
            or dynamic_ttl_s <= 0.0
            or post_dynamic_static_confirmation_s < 0.0
            or reversible_static_ttl_s < 0.0
        ):
            raise ValueError("invalid temporal voxel metric parameter")
        if (static_confirmation_hits < 2 or free_confirmation_rays < 1
                or dynamic_confirmation_hits < 1 or maximum_voxels < 1):
            raise ValueError("invalid temporal voxel count parameter")
        if not 0.0 < dynamic_clear_threshold < dynamic_occupied_threshold < 1.0:
            raise ValueError("dynamic thresholds must satisfy 0 < clear < occupied < 1")
        self.voxel_size_m = float(voxel_size_m)
        self.dynamic_ttl_s = float(dynamic_ttl_s)
        self.static_confirmation_hits = int(static_confirmation_hits)
        self.free_confirmation_rays = int(free_confirmation_rays)
        self.dynamic_occupied_threshold = float(dynamic_occupied_threshold)
        self.dynamic_clear_threshold = float(dynamic_clear_threshold)
        self.dynamic_confirmation_hits = int(dynamic_confirmation_hits)
        self.post_dynamic_static_confirmation_s = float(
            post_dynamic_static_confirmation_s
        )
        self.reversible_static_ttl_s = float(reversible_static_ttl_s)
        self.maximum_voxels = int(maximum_voxels)
        self._voxels: dict[tuple[int, int, int], DynamicVoxel] = {}
        self._free_columns: dict[tuple[int, int], int] = {}
        self._last_update_s = -math.inf
        self.update_sequence = 0
        self.last_update_timing_ms: dict[str, float] = {}

    def _key(self, point: np.ndarray) -> tuple[int, int, int]:
        return tuple(int(value) for value in np.floor(point / self.voxel_size_m))

    def _center(self, key: tuple[int, int, int]) -> np.ndarray:
        return (np.asarray(key, dtype=np.float64) + 0.5) * self.voxel_size_m

    @staticmethod
    def _points(value: np.ndarray, name: str) -> np.ndarray:
        array = np.asarray(value, dtype=np.float64)
        if array.ndim != 2 or array.shape[1] != 3:
            raise ValueError(f"{name} must have shape (N,3)")
        return array[np.isfinite(array).all(axis=1)]

    def _voxel(self, key: tuple[int, int, int], now_s: float) -> DynamicVoxel:
        voxel = self._voxels.get(key)
        if voxel is None:
            voxel = DynamicVoxel(last_decay_s=float(now_s))
            self._voxels[key] = voxel
        return voxel

    @staticmethod
    def _neighbors(key: tuple[int, int, int]):
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    yield key[0] + dx, key[1] + dy, key[2] + dz

    def decay(self, now_s: float) -> None:
        if not math.isfinite(now_s) or now_s < 0.0:
            raise ValueError("invalid map timestamp")
        for voxel in self._voxels.values():
            if (
                voxel.reversible_static
                and self.reversible_static_ttl_s > 0.0
                and now_s - voxel.last_seen_s > self.reversible_static_ttl_s
            ):
                # This occupancy originated in the dynamic layer.  Remaining
                # at one pose can make it provisionally static, but absence of
                # continued observations must remove it even when no clearing
                # ray reaches the old surface (for example after an object
                # leaves behind the vehicle).
                voxel.static_confidence = 0.0
                voxel.hit_count = 0
                voxel.reversible_static = False
                voxel.path_promoted = False
                voxel.occupancy_logodds = min(voxel.occupancy_logodds, -0.7)
            if voxel.dynamic_confidence <= 0.0:
                voxel.last_decay_s = float(now_s)
                continue
            elapsed = max(0.0, float(now_s) - voxel.last_decay_s)
            voxel.dynamic_confidence *= math.exp(-elapsed / self.dynamic_ttl_s)
            voxel.last_decay_s = float(now_s)
            if voxel.dynamic_confidence < self.dynamic_clear_threshold:
                voxel.dynamic_confidence = 0.0
                voxel.dynamic_observations = 0
                voxel.dynamic_first_hit_s = -math.inf
                voxel.path_promoted = False
                if voxel.static_confidence < 0.5:
                    voxel.occupancy_logodds = min(voxel.occupancy_logodds, -0.7)

    def _free_keys(
        self, origin: np.ndarray, endpoints: np.ndarray
    ) -> set[tuple[int, int, int]]:
        if len(endpoints) == 0:
            return set()
        step_m = 0.75 * self.voxel_size_m
        deltas = endpoints - origin
        distances = np.linalg.norm(deltas, axis=1)
        valid = distances > self.voxel_size_m
        if not np.any(valid):
            return set()
        deltas = deltas[valid]
        distances = distances[valid]
        steps = np.maximum(
            1,
            np.floor((distances - self.voxel_size_m) / step_m).astype(np.int64),
        )
        maximum_steps = int(np.max(steps))
        if maximum_steps <= 1:
            return set()
        sample_groups = []
        for count in np.unique(steps):
            if count <= 1:
                continue
            # Keep the exact scalar linspace arithmetic at voxel boundaries,
            # while batching all rays that have the same traversal length.
            ratios = np.linspace(0.0, 1.0, int(count), endpoint=False)[1:]
            group = deltas[steps == count]
            sample_groups.append(
                origin + group[:, None, :] * ratios[None, :, None]
            )
        if not sample_groups:
            return set()
        samples = np.concatenate(
            [group.reshape((-1, 3)) for group in sample_groups], axis=0
        )
        keys = np.unique(
            np.floor(samples / self.voxel_size_m).astype(np.int64), axis=0
        )
        return {tuple(int(value) for value in row) for row in keys}

    def update_scan(
        self,
        origin_xyz: np.ndarray,
        endpoints_xyz: np.ndarray,
        now_s: float,
        *,
        maximum_rays: int = 700,
        minimum_range_m: float = 0.75,
        maximum_range_m: float = 12.0,
    ) -> dict[str, int]:
        processing_start_ns = time.perf_counter_ns()
        origin = np.asarray(origin_xyz, dtype=np.float64).reshape(3)
        if not np.isfinite(origin).all() or not math.isfinite(now_s) or now_s < 0.0:
            raise ValueError("invalid scan origin or timestamp")
        points = self._points(endpoints_xyz, "endpoints_xyz")
        deltas = points - origin
        ranges = np.linalg.norm(deltas, axis=1)
        if minimum_range_m < self.voxel_size_m or maximum_range_m <= minimum_range_m:
            raise ValueError("invalid scan range limits")
        points = points[(ranges >= minimum_range_m) & (ranges <= maximum_range_m)]
        # Every unique endpoint participates in occupied-hit evidence.  Only
        # the much more expensive free-space ray casting is subsampled.  If
        # endpoints were subsampled as well, a small moving object could
        # disappear intermittently even though it is present in every scan.
        endpoint_key_array = np.unique(
            np.floor(points / self.voxel_size_m).astype(np.int64), axis=0
        )
        endpoint_keys = {
            tuple(int(value) for value in row) for row in endpoint_key_array
        }
        ray_points = points
        if len(ray_points) > maximum_rays:
            indices = np.linspace(0, len(ray_points) - 1, maximum_rays, dtype=int)
            ray_points = ray_points[indices]
        prepared_ns = time.perf_counter_ns()
        self.decay(float(now_s))
        decayed_ns = time.perf_counter_ns()
        free_keys = self._free_keys(origin, ray_points) - endpoint_keys
        free_keys_done_ns = time.perf_counter_ns()
        # Free-space evidence for unseen cells is represented compactly by
        # ``_free_columns`` below.  Materialising a DynamicVoxel for every
        # traversed cell made a 600-ray scan grow the sparse occupied map to
        # tens of thousands of empty Python objects.  Only cells that already
        # carry occupied/dynamic evidence need per-voxel clearing updates.
        # ``set.intersection`` performs that selection in C and preserves the
        # clearing semantics for baseline, reversible-static and dynamic
        # voxels.
        existing_free_keys = free_keys.intersection(self._voxels)
        for key in existing_free_keys:
            voxel = self._voxels[key]
            voxel.free_count = min(65535, voxel.free_count + 1)
            voxel.consecutive_free_count = min(
                65535, voxel.consecutive_free_count + 1
            )
            voxel.last_free_s = float(now_s)
            if (
                not voxel.baseline_static
                and voxel.consecutive_free_count >= self.free_confirmation_rays
            ):
                # Static evidence acquired after the frozen pre-mission map is
                # reversible. Repeated free-space rays after the last hit mean
                # a stopped dynamic object has departed; retaining it would
                # create a permanent ghost obstacle in trajectory certification.
                voxel.static_confidence = 0.0
                voxel.hit_count = 0
                voxel.dynamic_observations = 0
                voxel.dynamic_first_hit_s = -math.inf
                voxel.path_promoted = False
                voxel.reversible_static = False
                voxel.occupancy_logodds = min(voxel.occupancy_logodds, -0.7)
            elif voxel.static_confidence < 0.5:
                voxel.occupancy_logodds = max(-4.0, voxel.occupancy_logodds - 0.35)
        for column in {(key[0], key[1]) for key in free_keys}:
            self._free_columns[column] = min(
                65535, self._free_columns.get(column, 0) + 1
            )
        free_updates_done_ns = time.perf_counter_ns()
        dynamic_hits = 0
        static_hits = 0
        column_free_cache: dict[tuple[int, int], int] = {}
        for key in endpoint_keys:
            voxel = self._voxel(key, now_s)
            if voxel.baseline_static:
                # Frozen pre-mission structure is immutable by contract.  It
                # cannot enter the dynamic layer, so the 27-voxel free-space
                # neighborhood query below would only repeat a known answer
                # for most wall/floor returns in every scan.
                voxel.hit_count = min(65535, voxel.hit_count + 1)
                voxel.consecutive_free_count = 0
                voxel.last_seen_s = float(now_s)
                voxel.last_decay_s = float(now_s)
                voxel.occupancy_logodds = min(4.0, voxel.occupancy_logodds + 0.85)
                voxel.static_confidence = 1.0
                static_hits += 1
                continue
            had_static_seed = voxel.hit_count > 0 and voxel.dynamic_observations == 0
            # Ray casting and endpoint quantization rarely land in exactly the
            # same 25 cm cell.  A one-cell neighborhood is the bounded spatial
            # uncertainty implied by voxelization; it is not object dilation.
            column_key = (key[0], key[1])
            column_free = column_free_cache.get(column_key)
            if column_free is None:
                column_free = max(
                    (self._free_columns.get((key[0] + dx, key[1] + dy), 0)
                     for dx in (-1, 0, 1) for dy in (-1, 0, 1)),
                    default=0,
                )
                column_free_cache[column_key] = column_free
            # Columnar evidence is deliberately elevation-independent and is
            # normally sufficient.  The 27-cell query is retained as the
            # exact fallback for maps constructed before column evidence was
            # available, but avoided for the common confirmed-free case.
            if column_free >= self.free_confirmation_rays:
                was_confirmed_free = True
            else:
                neighborhood_free = max(
                    (self._voxels[item].free_count for item in self._neighbors(key)
                     if item in self._voxels),
                    default=0,
                )
                was_confirmed_free = (
                    neighborhood_free >= self.free_confirmation_rays
                )
            is_confirmed_static = voxel.static_confidence >= 0.5
            voxel.hit_count = min(65535, voxel.hit_count + 1)
            voxel.consecutive_free_count = 0
            voxel.last_seen_s = float(now_s)
            voxel.last_decay_s = float(now_s)
            voxel.occupancy_logodds = min(4.0, voxel.occupancy_logodds + 0.85)
            if (
                voxel.dynamic_history
                or (was_confirmed_free and not is_confirmed_static and not had_static_seed)
            ):
                continuous_track = now_s - voxel.last_dynamic_hit_s <= 0.60
                if continuous_track:
                    voxel.dynamic_observations = min(65535, voxel.dynamic_observations + 1)
                else:
                    voxel.dynamic_observations = 1
                    if not math.isfinite(voxel.dynamic_first_hit_s):
                        voxel.dynamic_first_hit_s = float(now_s)
                voxel.last_dynamic_hit_s = float(now_s)
                voxel.dynamic_confidence = max(
                    voxel.dynamic_confidence,
                    min(1.0, voxel.dynamic_observations / float(self.dynamic_confirmation_hits)),
                )
                voxel.static_confidence = 0.0
                voxel.dynamic_history = True
                voxel.reversible_static = False
                dynamic_hits += 1
                static_dwell_complete = bool(
                    math.isfinite(voxel.dynamic_first_hit_s)
                    and now_s - voxel.dynamic_first_hit_s
                    >= self.post_dynamic_static_confirmation_s
                )
                if (
                    voxel.hit_count >= self.static_confirmation_hits
                    and static_dwell_complete
                ):
                    # Free-to-occupied is the conservative first classification,
                    # but it is not proof that an object is still moving. A
                    # surface remaining in one map voxel for the complete
                    # confirmation horizon transfers to the reversible
                    # post-baseline static layer. Moving objects do not retain
                    # this evidence because intervening free rays reset the hit
                    # count. The voxel is not frozen, so it also cannot leave a
                    # permanent ghost after departure.
                    voxel.static_confidence = 1.0
                    voxel.dynamic_confidence = 0.0
                    voxel.dynamic_observations = 0
                    voxel.dynamic_first_hit_s = -math.inf
                    voxel.path_promoted = False
                    voxel.dynamic_history = False
                    voxel.reversible_static = True
                    dynamic_hits -= 1
                    static_hits += 1
            else:
                voxel.static_confidence = min(
                    1.0, voxel.hit_count / float(self.static_confirmation_hits)
                )
                static_hits += 1
        endpoints_done_ns = time.perf_counter_ns()
        self.update_sequence += 1
        self._last_update_s = float(now_s)
        self._prune(origin)
        pruned_ns = time.perf_counter_ns()
        self.last_update_timing_ms = {
            "prepare_ms": 1.0e-6 * (prepared_ns - processing_start_ns),
            "decay_ms": 1.0e-6 * (decayed_ns - prepared_ns),
            "free_key_generation_ms": 1.0e-6 * (
                free_keys_done_ns - decayed_ns
            ),
            "free_voxel_updates_ms": 1.0e-6 * (
                free_updates_done_ns - free_keys_done_ns
            ),
            "endpoint_updates_ms": 1.0e-6 * (
                endpoints_done_ns - free_updates_done_ns
            ),
            "prune_ms": 1.0e-6 * (pruned_ns - endpoints_done_ns),
            "total_ms": 1.0e-6 * (pruned_ns - processing_start_ns),
        }
        return {
            "rays": len(ray_points),
            "endpoints": len(endpoint_keys),
            "free_voxels": len(free_keys),
            "dynamic_hits": dynamic_hits,
            "static_hits": static_hits,
        }

    def _prune(self, origin: np.ndarray) -> None:
        overflow = len(self._voxels) - self.maximum_voxels
        if overflow <= 0:
            return
        removable = sorted(
            self._voxels,
            key=lambda key: (
                self._voxels[key].static_confidence >= 0.5,
                self._voxels[key].last_seen_s,
                -float(np.linalg.norm(self._center(key) - origin)),
            ),
        )
        for key in removable[:overflow]:
            del self._voxels[key]

    def promote_dynamic(self, points: np.ndarray, now_s: float) -> int:
        """Promote raw LiDAR hits inside a certified-free planned sweep.

        The caller owns the trajectory query; this method only enforces that
        confirmed static structure can never be promoted into the TTL layer.
        """
        array = self._points(points, "points")
        promoted = 0
        for key in {self._key(point) for point in array}:
            voxel = self._voxel(key, now_s)
            if voxel.baseline_static or voxel.static_confidence >= 0.5:
                continue
            voxel.static_confidence = 0.0
            voxel.reversible_static = False
            voxel.dynamic_observations = max(
                voxel.dynamic_observations, self.dynamic_confirmation_hits
            )
            if not math.isfinite(voxel.dynamic_first_hit_s):
                voxel.dynamic_first_hit_s = float(now_s)
            voxel.dynamic_confidence = 1.0
            voxel.last_dynamic_hit_s = float(now_s)
            voxel.last_seen_s = float(now_s)
            voxel.last_decay_s = float(now_s)
            voxel.occupancy_logodds = max(voxel.occupancy_logodds, 0.85)
            voxel.path_promoted = True
            voxel.dynamic_history = True
            promoted += 1
        return promoted

    def promotable_points(self, points: np.ndarray) -> np.ndarray:
        """Return free-to-occupied points not adjacent to frozen static map."""
        array = self._points(points, "points")
        if len(array) == 0:
            return np.empty((0, 3), dtype=np.float64)
        key_array = np.floor(array / self.voxel_size_m).astype(np.int64)
        unique_keys, first_indices = np.unique(
            key_array, axis=0, return_index=True
        )
        keep = []
        for row, point_index in zip(unique_keys, first_indices):
            key = tuple(int(value) for value in row)
            voxel = self._voxels.get(key)
            # Only a free-to-occupied voxel (or one already promoted in the
            # path) may enter the TTL layer. Newly revealed geometry with no
            # prior free evidence is static/unknown and must be routed around,
            # not forgotten as if it were a moving object.
            free_to_occupied = bool(
                voxel is not None
                and (voxel.dynamic_confidence > 0.0 or voxel.path_promoted)
            )
            if not free_to_occupied:
                continue
            confirmed_static_nearby = any(
                (key[0] + dx, key[1] + dy, key[2] + dz) in self._voxels
                and self._voxels[
                    (key[0] + dx, key[1] + dy, key[2] + dz)
                ].baseline_static
                for dx in range(-2, 3)
                for dy in range(-2, 3)
                for dz in range(-2, 3)
            )
            if not confirmed_static_nearby:
                keep.append(array[int(point_index)])
        return np.stack(keep) if keep else np.empty((0, 3), dtype=np.float64)

    def freeze_static_baseline(self) -> int:
        """Freeze all pre-mission occupied observations as immutable static."""
        count = 0
        for voxel in self._voxels.values():
            if voxel.hit_count <= 0:
                continue
            voxel.static_confidence = 1.0
            voxel.baseline_static = True
            voxel.reversible_static = False
            voxel.dynamic_history = False
            voxel.dynamic_confidence = 0.0
            voxel.dynamic_observations = 0
            voxel.dynamic_first_hit_s = -math.inf
            count += 1
        return count

    def dynamic_points(self) -> np.ndarray:
        points = [
            self._center(key) for key, voxel in self._voxels.items()
            if voxel.dynamic_confidence >= self.dynamic_occupied_threshold
            and voxel.static_confidence < 0.5
        ]
        return np.stack(points) if points else np.empty((0, 3), dtype=np.float64)

    def path_dynamic_points(self) -> np.ndarray:
        """Return TTL-live dynamics explicitly certified inside the path sweep."""
        points = [
            self._center(key) for key, voxel in self._voxels.items()
            if voxel.path_promoted
            and voxel.dynamic_confidence >= self.dynamic_occupied_threshold
            and voxel.static_confidence < 0.5
        ]
        return np.stack(points) if points else np.empty((0, 3), dtype=np.float64)

    def status_snapshot(self) -> tuple[dict[str, float | int], np.ndarray]:
        """Return map statistics and path dynamics from one consistent pass."""
        static_count = 0
        baseline_static_count = 0
        dynamic_count = 0
        reversible_static_count = 0
        path_dynamic_count = 0
        dynamic_peak = 0.0
        path_dynamic_keys: list[tuple[int, int, int]] = []
        for key, voxel in self._voxels.items():
            is_static = voxel.static_confidence >= 0.5 and not voxel.dynamic_history
            is_dynamic = (
                voxel.dynamic_confidence >= self.dynamic_occupied_threshold
                and voxel.static_confidence < 0.5
            )
            is_path_dynamic = is_dynamic and voxel.path_promoted
            static_count += int(is_static)
            baseline_static_count += int(voxel.baseline_static)
            dynamic_count += int(is_dynamic)
            reversible_static_count += int(voxel.reversible_static)
            path_dynamic_count += int(is_path_dynamic)
            if is_path_dynamic:
                path_dynamic_keys.append(key)
            if voxel.static_confidence < 0.5:
                dynamic_peak = max(dynamic_peak, voxel.dynamic_confidence)
        points = (
            np.stack([self._center(key) for key in path_dynamic_keys])
            if path_dynamic_keys else np.empty((0, 3), dtype=np.float64)
        )
        return (
            {
                "update_sequence": self.update_sequence,
                "voxel_count": len(self._voxels),
                "free_column_count": len(self._free_columns),
                "static_voxel_count": static_count,
                "baseline_static_voxel_count": baseline_static_count,
                "dynamic_voxel_count": dynamic_count,
                "reversible_static_voxel_count": reversible_static_count,
                "path_dynamic_voxel_count": path_dynamic_count,
                "dynamic_confidence_peak": dynamic_peak,
                "last_update_s": self._last_update_s,
            },
            points,
        )

    def static_points(self) -> np.ndarray:
        points = [
            self._center(key) for key, voxel in self._voxels.items()
            if voxel.static_confidence >= 0.5
            and not voxel.dynamic_history
        ]
        return np.stack(points) if points else np.empty((0, 3), dtype=np.float64)

    def confidence_points(self) -> tuple[np.ndarray, np.ndarray]:
        records = [
            (self._center(key), voxel.dynamic_confidence)
            for key, voxel in self._voxels.items()
            if voxel.dynamic_confidence > 0.0 and voxel.static_confidence < 0.5
        ]
        if not records:
            return np.empty((0, 3), dtype=np.float64), np.empty((0,), dtype=np.float64)
        return np.stack([item[0] for item in records]), np.asarray(
            [item[1] for item in records], dtype=np.float64
        )

    def stats(self) -> dict[str, float | int]:
        payload, _ = self.status_snapshot()
        return payload
