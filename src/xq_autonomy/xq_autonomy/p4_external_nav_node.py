"""Feed FAST-LIO odometry to MAVROS' MAVLink ODOMETRY plugin."""

from __future__ import annotations

import json
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def _stamp_s(message: Odometry) -> float:
    return float(message.header.stamp.sec) + 1e-9 * float(message.header.stamp.nanosec)


def _normalised_quaternion(message: Odometry) -> tuple[float, float, float, float]:
    q = message.pose.pose.orientation
    norm = math.sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("FAST-LIO quaternion is invalid")
    return q.w / norm, q.x / norm, q.y / norm, q.z / norm


def _world_to_body(
    vector: tuple[float, float, float], q: tuple[float, float, float, float]
) -> tuple[float, float, float]:
    """Rotate a map-frame vector by the inverse unit quaternion."""
    w, x, y, z = q
    vx, vy, vz = vector
    return (
        (1 - 2 * (y * y + z * z)) * vx + 2 * (x * y + w * z) * vy + 2 * (x * z - w * y) * vz,
        2 * (x * y - w * z) * vx + (1 - 2 * (x * x + z * z)) * vy + 2 * (y * z + w * x) * vz,
        2 * (x * z + w * y) * vx + 2 * (y * z - w * x) * vy + (1 - 2 * (x * x + y * y)) * vz,
    )


def _reported_body_velocity(message: Odometry) -> tuple[float, float, float] | None:
    """Return FAST-LIO's ESKF velocity when its covariance marks it available."""
    velocity = message.twist.twist.linear
    values = (float(velocity.x), float(velocity.y), float(velocity.z))
    diagonal = tuple(float(message.twist.covariance[index]) for index in (0, 7, 14))
    if not all(math.isfinite(value) and abs(value) <= 20.0 for value in values):
        return None
    if not any(math.isfinite(value) and value > 0.0 for value in diagonal):
        return None
    return values


class P4ExternalNavNode(Node):
    """Retimestamp LIO poses and derive body velocity for MAVROS ODOMETRY."""

    def __init__(self) -> None:
        super().__init__("xq_p4_external_nav")
        self.declare_parameter("input_topic", "/localization/odom")
        # MAVROS names these from the FCU viewpoint: `out` is ROS -> FCU,
        # while `in` is FCU -> ROS.
        self.declare_parameter("output_topic", "/uav1/mavros/odometry/out")
        self.declare_parameter("status_topic", "/xq/p4/extnav/status")
        self.declare_parameter("velocity_filter_alpha", 0.35)
        self.declare_parameter("position_variance_floor", 0.0025)
        self.declare_parameter("orientation_variance_floor", 0.0012)
        self.declare_parameter("velocity_variance", 0.04)
        self.declare_parameter("maximum_source_gap_s", 0.35)
        self.declare_parameter("minimum_healthy_rate_hz", 4.0)

        alpha = float(self.get_parameter("velocity_filter_alpha").value)
        if not 0.0 < alpha <= 1.0:
            raise ValueError("velocity_filter_alpha must be in (0, 1]")
        self.alpha = alpha
        self.position_floor = float(self.get_parameter("position_variance_floor").value)
        self.orientation_floor = float(self.get_parameter("orientation_variance_floor").value)
        self.velocity_variance = float(self.get_parameter("velocity_variance").value)
        self.maximum_gap = float(self.get_parameter("maximum_source_gap_s").value)

        reliable = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(
            Odometry, str(self.get_parameter("output_topic").value), reliable
        )
        self.status_publisher = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("input_topic").value),
            self._odom_cb,
            reliable,
        )
        self.create_timer(1.0, self._status_cb)

        self.previous: tuple[float, tuple[float, float, float]] | None = None
        self.filtered_velocity = (0.0, 0.0, 0.0)
        self.sample_times: list[float] = []
        self.source_count = 0
        self.output_count = 0
        self.rejected_count = 0
        self.esekf_velocity_count = 0
        self.differenced_velocity_count = 0
        self.last_source_wall = 0.0
        self.last_source_stamp = 0.0
        self.get_logger().info(
            "P4 ExternalNav adapter: /localization/odom -> MAVROS ODOMETRY; no ground truth input"
        )

    @staticmethod
    def _covariance_with_floor(
        source: list[float] | tuple[float, ...], position_floor: float, orientation_floor: float
    ) -> list[float]:
        output = [float(value) if math.isfinite(float(value)) else 0.0 for value in source]
        for index, floor in ((0, position_floor), (7, position_floor), (14, position_floor),
                             (21, orientation_floor), (28, orientation_floor), (35, orientation_floor)):
            output[index] = max(output[index], floor)
        return output

    def _odom_cb(self, source: Odometry) -> None:
        try:
            q = _normalised_quaternion(source)
            position = source.pose.pose.position
            xyz = (float(position.x), float(position.y), float(position.z))
            stamp = _stamp_s(source)
            if not all(math.isfinite(value) for value in (*xyz, stamp)):
                raise ValueError("non-finite FAST-LIO pose")
        except ValueError as exc:
            self.rejected_count += 1
            self.get_logger().error(str(exc))
            return

        reported_velocity = _reported_body_velocity(source)
        body_velocity = reported_velocity if reported_velocity is not None else self.filtered_velocity
        if reported_velocity is not None:
            # The ESKF velocity is time-aligned with this pose and needs no
            # additional low-pass filter; filtering here adds destabilising
            # phase lag to ArduPilot's position controller.
            self.filtered_velocity = reported_velocity
            self.esekf_velocity_count += 1
        elif self.previous is not None:
            previous_stamp, previous_xyz = self.previous
            dt = stamp - previous_stamp
            if 0.0 < dt <= self.maximum_gap:
                map_velocity = tuple((xyz[i] - previous_xyz[i]) / dt for i in range(3))
                raw_body = _world_to_body(map_velocity, q)
                body_velocity = tuple(
                    self.alpha * raw_body[i] + (1.0 - self.alpha) * self.filtered_velocity[i]
                    for i in range(3)
                )
                if all(math.isfinite(value) and abs(value) <= 20.0 for value in body_velocity):
                    self.filtered_velocity = body_velocity
                    self.differenced_velocity_count += 1
                else:
                    self.rejected_count += 1
                    body_velocity = self.filtered_velocity

        self.previous = (stamp, xyz)
        self.source_count += 1
        self.last_source_wall = time.monotonic()
        self.last_source_stamp = stamp
        self.sample_times.append(self.last_source_wall)
        cutoff = self.last_source_wall - 3.0
        while self.sample_times and self.sample_times[0] < cutoff:
            self.sample_times.pop(0)

        output = Odometry()
        # MAVROS runs on wall time even though the source estimator follows
        # Gazebo /clock.  Arrival retimestamping prevents a boot-time epoch mix.
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = "map"
        output.child_frame_id = "base_link"
        output.pose.pose = source.pose.pose
        output.pose.covariance = self._covariance_with_floor(
            source.pose.covariance, self.position_floor, self.orientation_floor
        )
        output.twist.twist.linear.x = body_velocity[0]
        output.twist.twist.linear.y = body_velocity[1]
        output.twist.twist.linear.z = body_velocity[2]
        for index in (0, 7, 14, 21, 28, 35):
            output.twist.covariance[index] = self.velocity_variance
        self.publisher.publish(output)
        self.output_count += 1

    def _status_cb(self) -> None:
        now = time.monotonic()
        source_age = now - self.last_source_wall if self.last_source_wall else math.inf
        span = self.sample_times[-1] - self.sample_times[0] if len(self.sample_times) > 1 else 0.0
        rate = (len(self.sample_times) - 1) / span if span > 0.0 else 0.0
        minimum_rate = float(self.get_parameter("minimum_healthy_rate_hz").value)
        subscribers = self.publisher.get_subscription_count()
        status = {
            "schema_version": 1,
            "healthy": source_age <= self.maximum_gap and rate >= minimum_rate and subscribers > 0,
            "source_count": self.source_count,
            "output_count": self.output_count,
            "rejected_count": self.rejected_count,
            "esekf_velocity_count": self.esekf_velocity_count,
            "differenced_velocity_count": self.differenced_velocity_count,
            "source_rate_hz": rate,
            "source_age_s": source_age if math.isfinite(source_age) else None,
            "source_stamp_s": self.last_source_stamp,
            "mavros_subscribers": subscribers,
            "ground_truth_subscribed": False,
        }
        message = String()
        message.data = json.dumps(status, separators=(",", ":"))
        self.status_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P4ExternalNavNode()
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
