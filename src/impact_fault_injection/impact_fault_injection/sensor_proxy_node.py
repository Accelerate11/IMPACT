"""In-process ROS data-path proxy that applies P14 sensor/link faults."""

from __future__ import annotations

from collections import deque
from copy import deepcopy
import json
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String
from xq_sim_interfaces.msg import FaultEvent

from .fault_model import ActiveFaultSet, FaultSpec


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _write_stamp_ns(stamp, value: int) -> None:
    value = max(0, int(value))
    stamp.sec = value // 1_000_000_000
    stamp.nanosec = value % 1_000_000_000


class SensorFaultProxyNode(Node):
    def __init__(self) -> None:
        super().__init__("impact_sensor_proxy")
        self.faults = ActiveFaultSet()
        self._delayed_odom: deque[tuple[float, Odometry]] = deque()
        self._last_output_stamp_ns = {"lidar": 0, "imu": 0}
        self._ground_windows: dict[str, dict[str, int]] = {}
        self._counters = {
            "lidar_received": 0, "lidar_dropped": 0,
            "imu_received": 0, "imu_dropped": 0,
            "jittered_messages": 0, "odom_delayed": 0,
            "covariance_inflated": 0, "ground_packets_sent": 0,
            "ground_packets_dropped": 0,
        }
        sensor_in = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(
            depth=50, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.lidar_pub = self.create_publisher(PointCloud2, "/livox/lidar", reliable)
        self.imu_pub = self.create_publisher(Imu, "/livox/imu", reliable)
        self.odom_pub = self.create_publisher(Odometry, "/localization/odom", reliable)
        self.status_pub = self.create_publisher(String, "/impact/fault_proxy_status", latched)
        self.create_subscription(PointCloud2, "/impact/raw/lidar", self._lidar_cb, sensor_in)
        self.create_subscription(Imu, "/impact/raw/imu", self._imu_cb, sensor_in)
        self.create_subscription(Odometry, "/impact/raw/odom", self._odom_cb, reliable)
        self.create_subscription(FaultEvent, "/impact/fault_event", self._fault_cb, latched)
        self.create_timer(0.01, self._release_odom)
        self.create_timer(0.10, self._ground_packet)
        self.create_timer(0.20, self._publish_status)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _fault_cb(self, message: FaultEvent) -> None:
        now_s = self._now_s()
        self.faults.add(FaultSpec(
            fault_id=message.fault_id,
            fault_type=message.target_module,
            start_time_s=0.0,
            duration_s=float(message.duration_s),
            severity=float(message.severity),
            seed=int(message.seed),
        ), now_s)

    def _jitter(self, message, stream: str, now_s: float):
        active = self.faults.active(now_s, "timestamp_jitter")
        if not active:
            self._last_output_stamp_ns[stream] = max(
                self._last_output_stamp_ns[stream], _stamp_ns(message.header.stamp)
            )
            return message
        spec, _ = active[0]
        output = deepcopy(message)
        delay_ns = int(1.0e9 * spec.severity * self.faults.random(spec.fault_id))
        candidate = _stamp_ns(output.header.stamp) - delay_ns
        # The perturbation changes age without publishing a backwards clock/stamp stream.
        candidate = max(candidate, self._last_output_stamp_ns[stream] + 1)
        _write_stamp_ns(output.header.stamp, candidate)
        self._last_output_stamp_ns[stream] = candidate
        self._counters["jittered_messages"] += 1
        return output

    def _lidar_cb(self, message: PointCloud2) -> None:
        now_s = self._now_s()
        self._counters["lidar_received"] += 1
        if self.faults.has(now_s, "lidar_dropout"):
            self._counters["lidar_dropped"] += 1
            return
        self.lidar_pub.publish(self._jitter(message, "lidar", now_s))

    def _imu_cb(self, message: Imu) -> None:
        now_s = self._now_s()
        self._counters["imu_received"] += 1
        if self.faults.has(now_s, "imu_dropout"):
            self._counters["imu_dropped"] += 1
            return
        self.imu_pub.publish(self._jitter(message, "imu", now_s))

    def _odom_cb(self, message: Odometry) -> None:
        now_s = self._now_s()
        output = deepcopy(message)
        if self.faults.has(now_s, "localization_covariance_inflation"):
            sigma = self.faults.severity(
                now_s, "localization_covariance_inflation", default=0.25
            )
            for index in (0, 7, 14):
                output.pose.covariance[index] = max(output.pose.covariance[index], sigma * sigma)
            self._counters["covariance_inflated"] += 1
        if self.faults.has(now_s, "odom_delay"):
            delay_s = self.faults.severity(now_s, "odom_delay", default=0.15)
            self._delayed_odom.append((now_s + delay_s, output))
            self._counters["odom_delayed"] += 1
        else:
            self.odom_pub.publish(output)

    def _release_odom(self) -> None:
        now_s = self._now_s()
        while self._delayed_odom and self._delayed_odom[0][0] <= now_s:
            _, message = self._delayed_odom.popleft()
            self.odom_pub.publish(message)

    def _ground_packet(self) -> None:
        now_s = self._now_s()
        self._counters["ground_packets_sent"] += 1
        active = self.faults.active(now_s, "20_percent_packet_loss")
        if active:
            spec, _ = active[0]
            window = self._ground_windows.setdefault(spec.fault_id, {"sent": 0, "dropped": 0})
            window["sent"] += 1
            if self.faults.random(spec.fault_id) < spec.severity:
                self._counters["ground_packets_dropped"] += 1
                window["dropped"] += 1

    def _publish_status(self) -> None:
        now_s = self._now_s()
        sent = self._counters["ground_packets_sent"]
        dropped = self._counters["ground_packets_dropped"]
        payload = {
            "stamp_s": now_s,
            "active_fault_ids": self.faults.ids(now_s),
            "counters": dict(self._counters),
            "ground_packet_loss_ratio": dropped / sent if sent else 0.0,
            "ground_fault_windows": {
                key: {
                    **value,
                    "loss_ratio": value["dropped"] / value["sent"] if value["sent"] else 0.0,
                }
                for key, value in self._ground_windows.items()
            },
            "ground_truth_used": False,
        }
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        self.status_pub.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SensorFaultProxyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
