"""P8 static-obstacle Alert Limit over the current EGO B-spline."""

from __future__ import annotations

import json
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from traj_utils.msg import Bspline
from xq_sim_interfaces.msg import AlertLimit

from .alert_limit import compute_alert_limit, sample_bspline


def _cloud_xyz(message: PointCloud2) -> np.ndarray:
    fields = {field.name: field for field in message.fields}
    if not all(name in fields for name in ("x", "y", "z")):
        return np.empty((0, 3), dtype=np.float64)
    if any(fields[name].datatype != PointField.FLOAT32 for name in ("x", "y", "z")):
        return np.empty((0, 3), dtype=np.float64)
    endian = ">" if message.is_bigendian else "<"
    dtype = np.dtype(
        {
            "names": ("x", "y", "z"),
            "formats": (endian + "f4", endian + "f4", endian + "f4"),
            "offsets": tuple(fields[name].offset for name in ("x", "y", "z")),
            "itemsize": message.point_step,
        }
    )
    records = np.frombuffer(message.data, dtype=dtype, count=int(message.width * message.height))
    points = np.column_stack((records["x"], records["y"], records["z"])).astype(np.float64)
    return points[np.isfinite(points).all(axis=1)]


class P8AlertLimitNode(Node):
    def __init__(self) -> None:
        super().__init__("xq_p8_alert_limit")
        defaults = {
            "body_radius_m": 0.35,
            "base_reserve_m": 0.10,
            "tracking_reserve_m": 0.10,
            "dynamic_reserve_m": 0.0,
            "latency_p99_s": 0.10,
            "maximum_acceleration_mps2": 1.0,
            "trajectory_sample_interval_s": 0.20,
            "maximum_obstacle_points": 12000,
            "minimum_publish_wall_interval_s": 0.04,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        if abs(self._parameter("dynamic_reserve_m")) > 1.0e-12:
            raise ValueError("P8 v1 is static-obstacle only; dynamic_reserve_m must be zero")
        self._control_points: np.ndarray | None = None
        self._knots: np.ndarray | None = None
        self._degree = 0
        self._trajectory_start_s = 0.0
        self._trajectory_id = -1
        self._speed = 0.0
        self._last_publish_wall = 0.0
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=30,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.publisher = self.create_publisher(AlertLimit, "/integrity/alert_limit", 20)
        self.debug_publisher = self.create_publisher(String, "/integrity/alert_limit_debug", 10)
        self.create_subscription(Bspline, "/planning/bspline", self._trajectory_cb, qos)
        self.create_subscription(PointCloud2, "/xq/p5/cloud_map", self._cloud_cb, qos)
        self.create_subscription(Odometry, "/localization/odom", self._odom_cb, qos)
        self.get_logger().info(
            "P8 Alert Limit ready: EGO B-spline + static mapped cloud; no Ground Truth subscription"
        )

    def _parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _odom_cb(self, message: Odometry) -> None:
        velocity = message.twist.twist.linear
        speed = math.sqrt(velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z)
        if math.isfinite(speed):
            self._speed = speed

    def _trajectory_cb(self, message: Bspline) -> None:
        control_points = np.asarray(
            [(point.x, point.y, point.z) for point in message.pos_pts], dtype=float
        )
        try:
            knots = np.asarray(message.knots, dtype=float)
            degree = int(message.order)
            sample_bspline(
                control_points,
                knots,
                degree,
                self._parameter("trajectory_sample_interval_s"),
            )
            self._control_points = control_points
            self._knots = knots
            self._degree = degree
            self._trajectory_start_s = float(message.start_time.sec) + 1.0e-9 * float(message.start_time.nanosec)
            self._trajectory_id = int(message.traj_id)
        except ValueError as error:
            self.get_logger().warning(f"Rejected invalid B-spline: {error}")

    def _cloud_cb(self, message: PointCloud2) -> None:
        if self._control_points is None or self._knots is None:
            return
        minimum_interval = self._parameter("minimum_publish_wall_interval_s")
        if time.monotonic() - self._last_publish_wall < minimum_interval:
            return
        obstacles = _cloud_xyz(message)
        maximum = int(self.get_parameter("maximum_obstacle_points").value)
        if len(obstacles) == 0:
            return
        if len(obstacles) > maximum:
            stride = int(math.ceil(len(obstacles) / maximum))
            obstacles = obstacles[::stride]
        cloud_stamp_s = float(message.header.stamp.sec) + 1.0e-9 * float(message.header.stamp.nanosec)
        elapsed_s = max(0.0, cloud_stamp_s - self._trajectory_start_s)
        try:
            trajectory = sample_bspline(
                self._control_points,
                self._knots,
                self._degree,
                self._parameter("trajectory_sample_interval_s"),
                minimum_parameter_s=elapsed_s,
            )
        except ValueError:
            return
        result = compute_alert_limit(
            trajectory,
            obstacles,
            speed_mps=self._speed,
            latency_p99_s=self._parameter("latency_p99_s"),
            maximum_acceleration_mps2=self._parameter("maximum_acceleration_mps2"),
            body_radius_m=self._parameter("body_radius_m"),
            base_reserve_m=self._parameter("base_reserve_m"),
            tracking_reserve_m=self._parameter("tracking_reserve_m"),
            dynamic_reserve_m=self._parameter("dynamic_reserve_m"),
        )
        output = AlertLimit()
        output.header = message.header
        output.trajectory_id = self._trajectory_id
        output.trajectory_sample_count = result.trajectory_sample_count
        output.obstacle_point_count = result.obstacle_point_count
        output.critical_sample_point.x, output.critical_sample_point.y, output.critical_sample_point.z = result.critical_sample.tolist()
        output.nearest_obstacle_point.x, output.nearest_obstacle_point.y, output.nearest_obstacle_point.z = result.nearest_obstacle.tolist()
        output.obstacle_direction_map = result.obstacle_direction.tolist()
        output.geometric_clearance = result.geometric_clearance
        output.body_radius = self._parameter("body_radius_m")
        output.base_reserve = self._parameter("base_reserve_m")
        output.tracking_reserve = self._parameter("tracking_reserve_m")
        output.dynamic_reserve = self._parameter("dynamic_reserve_m")
        output.latency_reserve = result.latency_reserve
        output.speed = self._speed
        output.latency_p99 = self._parameter("latency_p99_s")
        output.maximum_acceleration = self._parameter("maximum_acceleration_mps2")
        output.alert_limit = result.alert_limit
        output.valid = result.geometric_clearance > 1.0e-9
        output.static_obstacles_only = True
        output.obstacle_source = "/xq/p5/cloud_map"
        self.publisher.publish(output)
        debug = String()
        debug.data = json.dumps(
            {
                "phase": "P8_ALERT_LIMIT",
                "trajectory_id": self._trajectory_id,
                "samples": result.trajectory_sample_count,
                "obstacles": result.obstacle_point_count,
                "clearance_m": result.geometric_clearance,
                "latency_reserve_m": result.latency_reserve,
                "alert_limit_m": result.alert_limit,
                "static_obstacles_only": True,
                "ground_truth_subscribed": False,
                "planner_feedback_enabled": False,
            },
            separators=(",", ":"),
        )
        self.debug_publisher.publish(debug)
        self._last_publish_wall = time.monotonic()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P8AlertLimitNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
