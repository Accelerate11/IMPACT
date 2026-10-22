"""Pixel-derived angular flow and covered-window gyro compensation, without ROS."""

from __future__ import annotations

import bisect
from collections import deque
from dataclasses import dataclass
import math

import cv2
import numpy as np


BODY_FROM_DOWN_OPTICAL = np.asarray(
    ((0.0, -1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.0, -1.0))
)


def _timestamp(value: int) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0


@dataclass(frozen=True)
class CameraModel:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    frame_id: str = "impact_down_optical"

    def __post_init__(self) -> None:
        if (
            not all(math.isfinite(v) for v in (self.fx, self.fy, self.cx, self.cy))
            or min(self.width, self.height) <= 0
            or min(self.fx, self.fy) <= 0.0
            or not 0.0 <= self.cx < self.width
            or not 0.0 <= self.cy < self.height
            or self.frame_id != "impact_down_optical"
        ):
            raise ValueError("invalid downward pinhole camera calibration")

    @property
    def matrix(self) -> np.ndarray:
        return np.asarray(((self.fx, 0.0, self.cx), (0.0, self.fy, self.cy), (0.0, 0.0, 1.0)))


class GyroBuffer:
    """Bounded body gyro history; interpolate only within fully covered intervals."""

    def __init__(self, maximum_gap_s: float = 0.02, maximum_samples: int = 800):
        if not math.isfinite(maximum_gap_s) or not 0.0 < maximum_gap_s <= 0.02:
            raise ValueError("maximum gyro gap must be in (0, 0.02] seconds")
        if maximum_samples < 2:
            raise ValueError("gyro history needs at least two samples")
        self.maximum_gap_ns = int(round(maximum_gap_s * 1.0e9))
        self.samples: deque[tuple[int, np.ndarray]] = deque(maxlen=maximum_samples)

    def append(self, stamp_ns: int, body_gyro_rad_s) -> bool:
        angular_velocity = np.asarray(body_gyro_rad_s, dtype=float)
        if not _timestamp(stamp_ns) or angular_velocity.shape != (3,) or not np.isfinite(angular_velocity).all():
            return False
        if self.samples and stamp_ns <= self.samples[-1][0]:
            return False
        optical_velocity = BODY_FROM_DOWN_OPTICAL.T @ angular_velocity
        self.samples.append((stamp_ns, optical_velocity))
        return True

    def integrate(self, start_ns: int, end_ns: int) -> tuple[np.ndarray | None, str]:
        if not _timestamp(start_ns) or not _timestamp(end_ns) or end_ns <= start_ns:
            return None, "INVALID_INTEGRATION_INTERVAL"
        if len(self.samples) < 2:
            return None, "GYRO_WINDOW_UNCOVERED"
        stamps = [sample[0] for sample in self.samples]
        if start_ns < stamps[0] or end_ns > stamps[-1]:
            return None, "GYRO_WINDOW_UNCOVERED"
        left = bisect.bisect_right(stamps, start_ns) - 1
        right = bisect.bisect_left(stamps, end_ns)
        section = list(self.samples)[left : right + 1]
        if any(b[0] - a[0] > self.maximum_gap_ns for a, b in zip(section, section[1:])):
            return None, "GYRO_GAP_EXCEEDED"
        # Subtract the interval origin before converting to floating point,
        # preserving sub-frame precision even with epoch nanosecond timestamps.
        sample_times = np.asarray([sample[0] - start_ns for sample in section], dtype=float) * 1.0e-9
        values = np.asarray([sample[1] for sample in section], dtype=float)
        knots_ns = [start_ns, *[stamp for stamp in stamps[left : right + 1] if start_ns < stamp < end_ns], end_ns]
        knots_s = np.asarray([stamp - start_ns for stamp in knots_ns], dtype=float) * 1.0e-9
        rates = np.column_stack([np.interp(knots_s, sample_times, values[:, axis]) for axis in range(3)])
        integral = np.sum(0.5 * (rates[:-1] + rates[1:]) * np.diff(knots_s)[:, None], axis=0)
        return integral, "OK"


def rotational_displacements(
    points_px: np.ndarray, gyro_integral_optical_rad: np.ndarray, camera: CameraModel,
) -> np.ndarray:
    """Predict old-to-new pixel motion of stationary rays under camera rotation."""
    pixels = np.asarray(points_px, dtype=float).reshape(-1, 2)
    integral = np.asarray(gyro_integral_optical_rad, dtype=float).reshape(3)
    if not np.isfinite(pixels).all() or not np.isfinite(integral).all():
        raise ValueError("nonfinite rotational flow input")
    rays = np.column_stack(((pixels[:, 0] - camera.cx) / camera.fx,
                            (pixels[:, 1] - camera.cy) / camera.fy, np.ones(len(pixels))))
    rotation, _ = cv2.Rodrigues(-integral)
    new_rays = (rotation @ rays.T).T
    if np.any(new_rays[:, 2] <= 1.0e-6):
        raise ValueError("rotation projects a tracked ray behind the camera")
    projected = np.column_stack((camera.fx * new_rays[:, 0] / new_rays[:, 2] + camera.cx,
                                 camera.fy * new_rays[:, 1] / new_rays[:, 2] + camera.cy))
    return projected - pixels


def measurement_record(start_ns: int, end_ns: int, reason: str) -> dict[str, object]:
    """All invalid records retain source time and explicitly have no metric scale."""
    return {
        "schema_version": 1,
        "acquisition_start_ns": start_ns,
        "acquisition_end_ns": end_ns,
        "dt_s": (end_ns - start_ns) * 1.0e-9,
        "frame_id": "impact_down_optical",
        "source_image_stamps_ns": [start_ns, end_ns],
        "raw_integrated_flow_rad": [0.0, 0.0],
        "gyro_integral_optical_rad": [0.0, 0.0, 0.0],
        "compensated_integrated_flow_rad": [0.0, 0.0],
        "gyro_valid": False,
        "track_count": 0,
        "inlier_count": 0,
        "quality": 0,
        "valid": False,
        "reason": reason,
        "metric_velocity_valid": False,
        "metric_velocity_mps": None,
        "scale_source": "unavailable",
        "dependencies": ["down_camera", "livox_imu"],
        "source_model": "gazebo_pixels_lk",
        "ground_truth_used": False,
    }


def estimate_angular_flow(
    previous: np.ndarray, current: np.ndarray, camera: CameraModel,
    start_ns: int, end_ns: int, gyro_integral_optical_rad: np.ndarray,
    minimum_features: int = 12,
) -> dict[str, object]:
    """Forward/backward LK tracks with a robust translation residual median.

    Returned raw axes are image displacement (du/fx, dv/fy), not the differently
    signed sensor rotation axes of a MAVLink optical-flow message. Quality is a
    tracking support heuristic, not a calibrated probability or integrity bound.
    """
    result = measurement_record(start_ns, end_ns, "INVALID_IMAGE_INTERVAL")
    if not _timestamp(start_ns) or not _timestamp(end_ns) or not 0 < end_ns - start_ns < 200_000_000:
        return result
    for image in (previous, current):
        if not isinstance(image, np.ndarray) or image.dtype != np.uint8 or image.shape != (camera.height, camera.width):
            result["reason"] = "INVALID_IMAGE_LAYOUT"
            return result
    gyro = np.asarray(gyro_integral_optical_rad, dtype=float)
    if gyro.shape != (3,) or not np.isfinite(gyro).all():
        result["reason"] = "INVALID_GYRO_INTEGRAL"
        return result
    result["gyro_valid"] = True
    result["gyro_integral_optical_rad"] = gyro.tolist()
    features = cv2.goodFeaturesToTrack(previous, maxCorners=150, qualityLevel=0.01,
                                      minDistance=5.0, blockSize=5)
    result["track_count"] = 0 if features is None else len(features)
    if features is None or len(features) < minimum_features:
        result["reason"] = "INSUFFICIENT_FEATURES"
        return result
    lk_options = dict(winSize=(21, 21), maxLevel=3,
                      criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    forward, forward_status, _ = cv2.calcOpticalFlowPyrLK(previous, current, features, None, **lk_options)
    if forward is None or forward_status is None:
        result["reason"] = "LK_FORWARD_FAILED"
        return result
    backward, backward_status, _ = cv2.calcOpticalFlowPyrLK(current, previous, forward, None, **lk_options)
    if backward is None or backward_status is None:
        result["reason"] = "LK_BACKWARD_FAILED"
        return result
    old, new, returned = features.reshape(-1, 2), forward.reshape(-1, 2), backward.reshape(-1, 2)
    usable = (forward_status.reshape(-1).astype(bool) & backward_status.reshape(-1).astype(bool)
              & np.isfinite(new).all(axis=1) & np.isfinite(returned).all(axis=1)
              & (np.linalg.norm(returned - old, axis=1) <= 1.0)
              & (new[:, 0] >= 0) & (new[:, 0] < camera.width)
              & (new[:, 1] >= 0) & (new[:, 1] < camera.height))
    old, new = old[usable], new[usable]
    if len(old) < minimum_features:
        result["reason"] = "INSUFFICIENT_TRACKS"
        return result
    try:
        rotation_px = rotational_displacements(old, gyro, camera)
    except ValueError:
        result["reason"] = "INVALID_ROTATION_PROJECTION"
        return result
    residual_px = new - old - rotation_px
    centre = np.median(residual_px, axis=0)
    residual_distance = np.linalg.norm(residual_px - centre, axis=1)
    median_distance = float(np.median(residual_distance))
    deviation = float(np.median(np.abs(residual_distance - median_distance)))
    inliers = residual_distance <= max(1.0, median_distance + 3.0 * 1.4826 * deviation)
    count = int(np.count_nonzero(inliers))
    result["inlier_count"] = count
    if count < minimum_features:
        result["reason"] = "INSUFFICIENT_INLIERS"
        return result
    focal = np.asarray((camera.fx, camera.fy))
    result.update({
        "raw_integrated_flow_rad": (np.median(new[inliers] - old[inliers], axis=0) / focal).tolist(),
        "compensated_integrated_flow_rad": (np.median(residual_px[inliers], axis=0) / focal).tolist(),
        "quality": max(1, min(255, int(round(255.0 * min(count / 50.0, 1.0) * count / len(features))))),
        "valid": True,
        "reason": "OK_ANGULAR_ONLY_NO_SCALE",
    })
    return result


@dataclass(frozen=True)
class _ImageFrame:
    stamp_ns: int
    pixels: np.ndarray
    received_monotonic_ns: int


class DownwardFlowProcessor:
    """Two-frame latest-image queue, waiting a bounded time for gyro coverage."""

    def __init__(self, camera: CameraModel, maximum_pending_s: float = 0.2):
        if not math.isfinite(maximum_pending_s) or maximum_pending_s <= 0.0:
            raise ValueError("pending timeout must be positive and finite")
        self.camera = camera
        self.gyro = GyroBuffer()
        self.maximum_pending_ns = int(round(maximum_pending_s * 1.0e9))
        self.previous: _ImageFrame | None = None
        self.pending: _ImageFrame | None = None
        self.last_image_stamp_ns = -1
        self.rejected_gyro_samples = 0

    def push_image(
        self, stamp_ns: int, gray: np.ndarray, frame_id: str, received_monotonic_ns: int,
    ) -> list[dict[str, object]]:
        start = self.previous.stamp_ns if self.previous is not None else stamp_ns
        if not _timestamp(stamp_ns) or not _timestamp(received_monotonic_ns):
            raise ValueError("image and monotonic timestamps must be nonnegative integers")
        if stamp_ns <= self.last_image_stamp_ns:
            return [measurement_record(start, stamp_ns, "IMAGE_STAMP_NOT_INCREASING")]
        self.last_image_stamp_ns = stamp_ns
        if frame_id != self.camera.frame_id:
            return [measurement_record(start, stamp_ns, "IMAGE_FRAME_MISMATCH")]
        if not isinstance(gray, np.ndarray) or gray.dtype != np.uint8 or gray.shape != (self.camera.height, self.camera.width):
            return [measurement_record(start, stamp_ns, "INVALID_IMAGE_LAYOUT")]
        frame = _ImageFrame(stamp_ns, np.array(gray, copy=True), received_monotonic_ns)
        results = []
        if self.pending is not None:
            # Try the previous pair with all currently available gyro first.
            results.extend(self.expire(received_monotonic_ns))
            if self.pending is not None:
                results.append(measurement_record(self.previous.stamp_ns, self.pending.stamp_ns,
                                                  "GYRO_WINDOW_UNCOVERED_IMAGE_SUPERSEDED"))
                self.previous, self.pending = self.pending, None
        if self.previous is None:
            self.previous = frame
            return results + [measurement_record(stamp_ns, stamp_ns, "INITIALIZING_FIRST_IMAGE")]
        if not 0 < stamp_ns - self.previous.stamp_ns < 200_000_000:
            results.append(measurement_record(self.previous.stamp_ns, stamp_ns, "INVALID_IMAGE_INTERVAL"))
            self.previous = frame
            return results
        self.pending = frame
        return results + self.expire(received_monotonic_ns)

    def push_gyro(self, stamp_ns: int, body_gyro_rad_s, now_monotonic_ns: int) -> list[dict[str, object]]:
        if not self.gyro.append(stamp_ns, body_gyro_rad_s):
            self.rejected_gyro_samples += 1
        return self.expire(now_monotonic_ns)

    def expire(self, now_monotonic_ns: int) -> list[dict[str, object]]:
        if self.previous is None or self.pending is None:
            return []
        start, end = self.previous.stamp_ns, self.pending.stamp_ns
        integral, reason = self.gyro.integrate(start, end)
        if integral is None:
            missing_start = bool(self.gyro.samples and self.gyro.samples[0][0] > start)
            expired = now_monotonic_ns - self.pending.received_monotonic_ns >= self.maximum_pending_ns
            if reason == "GYRO_WINDOW_UNCOVERED" and not missing_start and not expired:
                return []
            result = measurement_record(start, end, reason)
        else:
            result = estimate_angular_flow(self.previous.pixels, self.pending.pixels,
                                           self.camera, start, end, integral)
        self.previous, self.pending = self.pending, None
        return [result]
