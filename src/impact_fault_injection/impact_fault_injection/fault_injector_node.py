"""ROS publisher for reproducible P14 fault windows."""

from __future__ import annotations

from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from xq_sim_interfaces.msg import FaultEvent

from .fault_model import FaultSpec, SUPPORTED_FAULTS, load_schedule


class FaultInjectorNode(Node):
    def __init__(self) -> None:
        super().__init__("impact_fault_injector")
        self.declare_parameter("schedule_file", "")
        self.declare_parameter("fault", "")
        self.declare_parameter("start_time_s", 30.0)
        self.declare_parameter("duration_s", 1.0)
        self.declare_parameter("severity", 1.0)
        self.declare_parameter("seed", 20260828)
        schedule = str(self.get_parameter("schedule_file").value)
        if schedule:
            self.specs = load_schedule(Path(schedule))
        else:
            fault_type = str(self.get_parameter("fault").value)
            if fault_type not in SUPPORTED_FAULTS:
                raise ValueError(f"unsupported fault: {fault_type}")
            self.specs = [FaultSpec(
                fault_id=f"single_{fault_type}", fault_type=fault_type,
                start_time_s=float(self.get_parameter("start_time_s").value),
                duration_s=float(self.get_parameter("duration_s").value),
                severity=float(self.get_parameter("severity").value),
                seed=int(self.get_parameter("seed").value),
            )]
        self._start_s: float | None = None
        self._published: set[str] = set()
        qos = QoSProfile(
            depth=50, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(FaultEvent, "/impact/fault_event", qos)
        self.create_timer(0.02, self._tick)
        self.get_logger().info(f"P14 loaded {len(self.specs)} deterministic fault windows")

    def _tick(self) -> None:
        now_s = self.get_clock().now().nanoseconds * 1.0e-9
        if now_s <= 0.0:
            return
        if self._start_s is None:
            self._start_s = now_s
        elapsed_s = now_s - self._start_s
        for spec in self.specs:
            if spec.fault_id in self._published or elapsed_s < spec.start_time_s:
                continue
            message = FaultEvent()
            message.header.stamp = self.get_clock().now().to_msg()
            message.fault_id = spec.fault_id
            message.target_module = spec.fault_type
            message.action = "activate"
            message.severity = spec.severity
            message.duration_s = spec.duration_s
            message.seed = spec.seed
            self.publisher.publish(message)
            self._published.add(spec.fault_id)
            self.get_logger().warning(
                f"P14 fault={spec.fault_id} type={spec.fault_type} "
                f"duration={spec.duration_s:.2f}s seed={spec.seed}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FaultInjectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
