"""Continuous Mid-360-like sensor-contract validation for IMPACT Phase 2."""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu, PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass
class StreamStats:
    count: int = 0
    first_stamp_ns: int | None = None
    last_stamp_ns: int | None = None
    dt_count: int = 0
    dt_sum_s: float = 0.0
    dt_sq_sum_s: float = 0.0
    min_dt_s: float = math.inf
    max_dt_s: float = 0.0
    monotonic_violations: int = 0
    zero_stamp_count: int = 0

    def add(self, stamp_ns: int) -> None:
        self.count += 1
        if stamp_ns <= 0:
            self.zero_stamp_count += 1
        if self.first_stamp_ns is None:
            self.first_stamp_ns = stamp_ns
        if self.last_stamp_ns is not None:
            if stamp_ns <= self.last_stamp_ns:
                self.monotonic_violations += 1
            else:
                dt = (stamp_ns - self.last_stamp_ns) * 1e-9
                self.dt_count += 1
                self.dt_sum_s += dt
                self.dt_sq_sum_s += dt * dt
                self.min_dt_s = min(self.min_dt_s, dt)
                self.max_dt_s = max(self.max_dt_s, dt)
        self.last_stamp_ns = stamp_ns

    def report(self) -> dict[str, object]:
        stamp_span_s = 0.0
        if self.first_stamp_ns is not None and self.last_stamp_ns is not None:
            stamp_span_s = max(0.0, (self.last_stamp_ns - self.first_stamp_ns) * 1e-9)
        rate_hz = (self.count - 1) / stamp_span_s if self.count > 1 and stamp_span_s > 0 else 0.0
        mean_dt = self.dt_sum_s / self.dt_count if self.dt_count else None
        std_dt = None
        if self.dt_count:
            variance = max(0.0, self.dt_sq_sum_s / self.dt_count - mean_dt * mean_dt)
            std_dt = math.sqrt(variance)
        return {
            "count": self.count,
            "stamp_span_s": stamp_span_s,
            "rate_hz": rate_hz,
            "mean_dt_s": mean_dt,
            "std_dt_s": std_dt,
            "min_dt_s": self.min_dt_s if self.dt_count else None,
            "max_dt_s": self.max_dt_s if self.dt_count else None,
            "monotonic_violations": self.monotonic_violations,
            "zero_stamp_count": self.zero_stamp_count,
        }


def has_observed_irreversible_failure(
    lidar: StreamStats,
    imu: StreamStats,
    lidar_frame_mismatches: int,
    imu_frame_mismatches: int,
    nan_or_inf_violations: int,
    layout_violations: int,
) -> bool:
    """Return true only for contract failures observed in received samples."""
    return any(
        (
            lidar.monotonic_violations,
            imu.monotonic_violations,
            lidar.zero_stamp_count,
            imu.zero_stamp_count,
            lidar_frame_mismatches,
            imu_frame_mismatches,
            nan_or_inf_violations,
            layout_violations,
        )
    )


class P2SensorValidator(Node):
    def __init__(self) -> None:
        super().__init__("xq_p2_sensor_validator")
        self.declare_parameter("result_file", "")
        self.declare_parameter("minimum_duration_s", 600.0)
        self.declare_parameter("lidar_topic", "/livox/lidar")
        self.declare_parameter("imu_topic", "/livox/imu")
        self.declare_parameter("lidar_frame", "livox_frame")
        self.declare_parameter("imu_frame", "livox_imu")
        self.declare_parameter("base_frame", "xq_base_link")
        self.declare_parameter("expected_lidar_rate_hz", 10.0)
        self.declare_parameter("minimum_imu_rate_hz", 100.0)
        self.declare_parameter("maximum_lidar_gap_s", 0.25)
        self.declare_parameter("maximum_imu_gap_s", 0.05)

        lidar_topic = str(self.get_parameter("lidar_topic").value)
        imu_topic = str(self.get_parameter("imu_topic").value)
        lidar_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        imu_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PointCloud2, lidar_topic, self._lidar_cb, lidar_qos)
        self.create_subscription(Imu, imu_topic, self._imu_cb, imu_qos)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        self.lidar = StreamStats()
        self.imu = StreamStats()
        self.started_wall = time.monotonic()
        self.capture_started_wall: float | None = None
        self.lidar_fields: list[dict[str, object]] = []
        self.lidar_frame_mismatches = 0
        self.imu_frame_mismatches = 0
        self.nan_or_inf_violations = 0
        self.layout_violations = 0
        self.min_points: int | None = None
        self.max_points = 0
        self.tf_checks: dict[str, dict[str, object]] = {}
        self.last_status = "IN_PROGRESS"
        self.create_timer(1.0, self._timer_cb)
        self.get_logger().info(
            f"P2 validation started: {lidar_topic}, {imu_topic}, "
            f"duration={self.get_parameter('minimum_duration_s').value}s"
        )

    @staticmethod
    def _stamp_ns(message) -> int:
        return int(message.header.stamp.sec) * 1_000_000_000 + int(
            message.header.stamp.nanosec
        )

    def _start_capture_if_ready(self) -> None:
        if self.capture_started_wall is None and self.lidar.count and self.imu.count:
            self.capture_started_wall = time.monotonic()
            self.get_logger().info("P2 capture window started: both streams are present")

    def _point_array(self, msg: PointCloud2, field: PointField) -> np.ndarray:
        if field.datatype == PointField.FLOAT32:
            dtype = np.dtype(">f4" if msg.is_bigendian else "<f4")
        elif field.datatype == PointField.FLOAT64:
            dtype = np.dtype(">f8" if msg.is_bigendian else "<f8")
        else:
            raise ValueError(f"field {field.name} is not floating point")
        return np.ndarray(
            shape=(int(msg.height), int(msg.width)),
            dtype=dtype,
            buffer=memoryview(msg.data),
            offset=int(field.offset),
            strides=(int(msg.row_step), int(msg.point_step)),
        )

    def _lidar_cb(self, msg: PointCloud2) -> None:
        self.lidar.add(self._stamp_ns(msg))
        expected_frame = str(self.get_parameter("lidar_frame").value)
        if msg.header.frame_id != expected_frame:
            self.lidar_frame_mismatches += 1
        point_count = int(msg.width) * int(msg.height)
        self.min_points = point_count if self.min_points is None else min(self.min_points, point_count)
        self.max_points = max(self.max_points, point_count)

        fields = {field.name: field for field in msg.fields}
        if not self.lidar_fields:
            self.lidar_fields = [
                {
                    "name": field.name,
                    "offset": int(field.offset),
                    "datatype": int(field.datatype),
                    "count": int(field.count),
                }
                for field in msg.fields
            ]
        if not all(name in fields for name in ("x", "y", "z")):
            self.layout_violations += 1
        elif len(msg.data) < int(msg.row_step) * int(msg.height):
            self.layout_violations += 1
        else:
            try:
                finite = all(
                    bool(np.isfinite(self._point_array(msg, fields[name])).all())
                    for name in ("x", "y", "z")
                )
                if not finite:
                    self.nan_or_inf_violations += 1
            except (TypeError, ValueError, BufferError):
                self.layout_violations += 1
        self._start_capture_if_ready()

    def _imu_cb(self, msg: Imu) -> None:
        self.imu.add(self._stamp_ns(msg))
        if msg.header.frame_id != str(self.get_parameter("imu_frame").value):
            self.imu_frame_mismatches += 1
        values = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        )
        if not all(math.isfinite(value) for value in values):
            self.nan_or_inf_violations += 1
        self._start_capture_if_ready()

    def _check_tf(self, child_frame: str) -> None:
        base_frame = str(self.get_parameter("base_frame").value)
        try:
            transform = self.tf_buffer.lookup_transform(base_frame, child_frame, Time())
            t = transform.transform.translation
            q = transform.transform.rotation
            finite = all(
                math.isfinite(value)
                for value in (t.x, t.y, t.z, q.x, q.y, q.z, q.w)
            )
            norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
            self.tf_checks[child_frame] = {
                "available": True,
                "finite": finite,
                "quaternion_norm": norm,
                "translation_m": [t.x, t.y, t.z],
                "rotation_xyzw": [q.x, q.y, q.z, q.w],
                "valid": finite and abs(norm - 1.0) < 1e-3,
            }
        except TransformException as exc:
            self.tf_checks[child_frame] = {
                "available": False,
                "valid": False,
                "error": str(exc),
            }

    def _snapshot(self) -> dict[str, object]:
        lidar = self.lidar.report()
        imu = self.imu.report()
        elapsed = (
            time.monotonic() - self.capture_started_wall
            if self.capture_started_wall is not None
            else 0.0
        )
        target_lidar = float(self.get_parameter("expected_lidar_rate_hz").value)
        min_imu = float(self.get_parameter("minimum_imu_rate_hz").value)
        checks = {
            "duration": elapsed >= float(self.get_parameter("minimum_duration_s").value),
            "lidar_rate": target_lidar * 0.95 <= float(lidar["rate_hz"]) <= target_lidar * 1.05,
            "imu_rate": float(imu["rate_hz"]) >= min_imu,
            "lidar_gap": (
                lidar["max_dt_s"] is not None
                and float(lidar["max_dt_s"])
                <= float(self.get_parameter("maximum_lidar_gap_s").value)
            ),
            "imu_gap": (
                imu["max_dt_s"] is not None
                and float(imu["max_dt_s"])
                <= float(self.get_parameter("maximum_imu_gap_s").value)
            ),
            "timestamps_monotonic": (
                self.lidar.monotonic_violations == 0
                and self.imu.monotonic_violations == 0
                and self.lidar.zero_stamp_count == 0
                and self.imu.zero_stamp_count == 0
            ),
            "frames": self.lidar_frame_mismatches == 0 and self.imu_frame_mismatches == 0,
            "tf": bool(self.tf_checks)
            and all(bool(item.get("valid")) for item in self.tf_checks.values()),
            "finite_values": self.nan_or_inf_violations == 0,
            "point_layout": self.layout_violations == 0 and bool(self.lidar_fields),
            "point_count": self.min_points is not None and self.min_points >= 1000,
        }
        # Absence of the first lidar/IMU sample is a startup condition, not a
        # contract violation.  Only failures observed in received data are
        # irreversible and may terminate a run before the capture duration.
        irreversible_failure = has_observed_irreversible_failure(
            self.lidar,
            self.imu,
            self.lidar_frame_mismatches,
            self.imu_frame_mismatches,
            self.nan_or_inf_violations,
            self.layout_violations,
        )
        if irreversible_failure:
            status = "FAIL"
        elif checks["duration"]:
            status = "PASS" if all(checks.values()) else "FAIL"
        else:
            status = "IN_PROGRESS"
        return {
            "schema_version": 1,
            "gate": "P2_MID360_SENSOR_CONTRACT",
            "status": status,
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "capture_wall_duration_s": elapsed,
            "checks": checks,
            "lidar": {
                **lidar,
                "topic": str(self.get_parameter("lidar_topic").value),
                "frame": str(self.get_parameter("lidar_frame").value),
                "fields": self.lidar_fields,
                "min_points_per_frame": self.min_points,
                "max_points_per_frame": self.max_points,
                "frame_mismatches": self.lidar_frame_mismatches,
            },
            "imu": {
                **imu,
                "topic": str(self.get_parameter("imu_topic").value),
                "frame": str(self.get_parameter("imu_frame").value),
                "frame_mismatches": self.imu_frame_mismatches,
            },
            "tf": self.tf_checks,
            "nan_or_inf_violations": self.nan_or_inf_violations,
            "layout_violations": self.layout_violations,
        }

    def _write_snapshot(self) -> None:
        result_path = Path(str(self.get_parameter("result_file").value))
        if str(result_path) in ("", "."):
            return
        result = self._snapshot()
        result_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = result_path.with_suffix(result_path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        temporary.replace(result_path)
        if result["status"] != self.last_status:
            self.get_logger().info(f"P2 validation status -> {result['status']}")
            self.last_status = str(result["status"])

    def _timer_cb(self) -> None:
        self._check_tf(str(self.get_parameter("lidar_frame").value))
        self._check_tf(str(self.get_parameter("imu_frame").value))
        self._write_snapshot()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P2SensorValidator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._write_snapshot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
