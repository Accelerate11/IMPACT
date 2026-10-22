"""Dedicated down-camera angular flow; no odometry, truth, or height inputs."""

from __future__ import annotations

import json
import os
from pathlib import Path
import tempfile
import time

import cv2
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import String

from .downward_flow import CameraModel, DownwardFlowProcessor, GyroBuffer, measurement_record


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def camera_from_info(message: CameraInfo) -> CameraModel:
    matrix = np.asarray(message.k, dtype=float).reshape(3, 3)
    if (
        not np.isfinite(matrix).all()
        or not np.allclose(matrix[2], (0.0, 0.0, 1.0), atol=1.0e-9)
        or abs(matrix[0, 1]) > 1.0e-9
        or abs(matrix[1, 0]) > 1.0e-9
        or not np.isfinite(message.d).all()
        or any(abs(value) > 1.0e-9 for value in message.d)
    ):
        raise ValueError("flow requires calibrated, zero-distortion pinhole images")
    return CameraModel(int(message.width), int(message.height), float(matrix[0, 0]),
                       float(matrix[1, 1]), float(matrix[0, 2]), float(matrix[1, 2]),
                       message.header.frame_id)


def image_to_gray(message: Image) -> np.ndarray:
    if message.encoding != "rgb8" or min(message.width, message.height) <= 0:
        raise ValueError("down-camera input must be nonempty rgb8")
    if message.step < message.width * 3 or len(message.data) < message.step * message.height:
        raise ValueError("truncated or invalid RGB image row layout")
    rgb = np.ndarray((message.height, message.width, 3), dtype=np.uint8,
                     buffer=memoryview(message.data), strides=(message.step, 3, 1))
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)


class DownwardFlowNode(Node):
    def __init__(self) -> None:
        super().__init__("impact_downward_flow")
        # OpenCV's process-local worker pool otherwise competes with ROS/LIO on
        # the WSL host. No library code or environment configuration is patched.
        cv2.setNumThreads(1)
        self.declare_parameter("result_file", "")
        self.declare_parameter("status_file", "")
        self.camera: CameraModel | None = None
        self.processor: DownwardFlowProcessor | None = None
        self.gyro = GyroBuffer()
        self.last_image_stamp_ns = -1
        self.last_gyro_stamp_ns = -1
        self.last_reason = "WAIT_CAMERA_INFO"
        self.counts = {name: 0 for name in (
            "images_received", "images_rejected", "gyro_received", "gyro_rejected",
            "camera_info_received", "camera_info_rejected", "measurements_total",
            "valid_measurements", "invalid_measurements",
        )}
        self.reason_counts: dict[str, int] = {}
        sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        imu_qos = QoSProfile(depth=400, reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable = QoSProfile(depth=30, reliability=ReliabilityPolicy.RELIABLE)
        self.measurement_publisher = self.create_publisher(String, "/impact/r2/flow/measurement", reliable)
        self.status_publisher = self.create_publisher(String, "/impact/r2/flow/status", reliable)
        self.create_subscription(Image, "/impact/r2/down/image_raw", self._image_cb, sensor)
        self.create_subscription(CameraInfo, "/impact/r2/down/camera_info", self._camera_cb, sensor)
        self.create_subscription(Imu, "/livox/imu", self._imu_cb, imu_qos)
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.02, self._expire, clock=self._steady_clock)
        self.create_timer(1.0, self._status, clock=self._steady_clock)
        self.get_logger().info("Downward pixel/gyro flow ready; angular measurements only, metric scale unavailable")

    def _camera_cb(self, message: CameraInfo) -> None:
        self.counts["camera_info_received"] += 1
        try:
            camera = camera_from_info(message)
        except (TypeError, ValueError):
            self.counts["camera_info_rejected"] += 1
            self.last_reason = "INVALID_CAMERA_INFO"
            self.camera = self.processor = None
            return
        if camera != self.camera:
            self.camera = camera
            self.processor = DownwardFlowProcessor(camera)
            self.processor.gyro = self.gyro
            self.last_reason = "WAIT_IMAGE_PAIR"

    def _image_cb(self, message: Image) -> None:
        self.counts["images_received"] += 1
        stamp = _stamp_ns(message.header.stamp)
        if stamp < 0 or stamp <= self.last_image_stamp_ns:
            # Replayed headers produce no new frame measurement. The status
            # retains the rejection, without fabricating acquisition times.
            self.counts["images_rejected"] += 1
            self.last_reason = "IMAGE_STAMP_NOT_INCREASING"
            return
        self.last_image_stamp_ns = stamp
        if self.processor is None:
            self.counts["images_rejected"] += 1
            self._publish([measurement_record(stamp, stamp, "CAMERA_INFO_UNAVAILABLE")])
            return
        try:
            gray = image_to_gray(message)
            records = self.processor.push_image(stamp, gray, message.header.frame_id, time.monotonic_ns())
        except (TypeError, ValueError, BufferError, cv2.error):
            self.counts["images_rejected"] += 1
            records = [measurement_record(stamp, stamp, "INVALID_IMAGE_LAYOUT")]
        self._publish(records)

    def _imu_cb(self, message: Imu) -> None:
        self.counts["gyro_received"] += 1
        stamp = _stamp_ns(message.header.stamp)
        omega = message.angular_velocity
        if (
            message.header.frame_id != "livox_imu"
            or message.angular_velocity_covariance[0] < 0.0
            or not self.gyro.append(stamp, (omega.x, omega.y, omega.z))
        ):
            self.counts["gyro_rejected"] += 1
            return
        self.last_gyro_stamp_ns = stamp
        self._expire()

    def _expire(self) -> None:
        if self.processor is not None:
            self._publish(self.processor.expire(time.monotonic_ns()))

    def _publish(self, records: list[dict[str, object]]) -> None:
        for record in records:
            self.counts["measurements_total"] += 1
            self.counts["valid_measurements" if record["valid"] else "invalid_measurements"] += 1
            self.last_reason = str(record["reason"])
            self.reason_counts[self.last_reason] = self.reason_counts.get(self.last_reason, 0) + 1
            payload = json.dumps(record, allow_nan=False, separators=(",", ":"))
            self.measurement_publisher.publish(String(data=payload))
            self._save("result_file", record)

    def _status(self) -> None:
        payload = {
            "schema_version": 1,
            "counters": dict(self.counts),
            "measurement_reasons": dict(self.reason_counts),
            "last_reason": self.last_reason,
            "last_image_acquisition_ns": self.last_image_stamp_ns,
            "last_gyro_acquisition_ns": self.last_gyro_stamp_ns,
            "acquisition_clock_domain": "ros_sim",
            "receipt_clock_domain": "monotonic",
            "camera_calibrated": self.camera is not None,
            "pending_image_count": int(self.processor is not None and self.processor.pending is not None),
            "maximum_pending_wall_s": 0.2,
            "gyro_buffer_samples": len(self.gyro.samples),
            "metric_velocity_valid": False,
            "scale_source": "unavailable",
            "ground_truth_used": False,
            "quality_model": "feature_support_heuristic_not_integrity_probability",
        }
        self.status_publisher.publish(String(data=json.dumps(payload, allow_nan=False)))
        self._save("status_file", payload)

    def _save(self, parameter: str, payload: dict[str, object]) -> None:
        destination = str(self.get_parameter(parameter).value).strip()
        if not destination:
            return
        path = Path(destination).expanduser()
        temporary = None
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                             prefix=f".{path.name}.", suffix=".tmp", delete=False) as stream:
                temporary = Path(stream.name)
                json.dump(payload, stream, allow_nan=False, indent=2)
                stream.write("\n")
            os.replace(temporary, path)
        except OSError as error:
            self.get_logger().error(f"Cannot save flow diagnostic: {error}")
        finally:
            if temporary is not None:
                try:
                    temporary.unlink(missing_ok=True)
                except OSError:
                    pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DownwardFlowNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._status()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
