from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, List

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from xq_sim_interfaces.msg import FaultEvent


class XqFaultInjector(Node):
    """Publish a deterministic fault timeline using simulation time."""

    def __init__(self) -> None:
        super().__init__("xq_fault_injector")
        self.declare_parameter("schedule_file", "")
        self.declare_parameter("seed", 20260820)
        schedule = Path(str(self.get_parameter("schedule_file").value)).expanduser()
        if not schedule.is_file():
            raise FileNotFoundError(f"fault schedule not found: {schedule}")
        data = json.loads(schedule.read_text(encoding="utf-8"))
        self.events: List[Dict[str, object]] = sorted(data.get("events", []), key=lambda item: float(item["at_s"]))
        self._published: set[str] = set()
        self._start_s: float | None = None
        self.publisher = self.create_publisher(FaultEvent, "/xq/test/fault_event", 20)
        self.create_timer(0.05, self._tick)
        self.get_logger().info(f"Loaded {len(self.events)} fault events from {schedule}")

    def _tick(self) -> None:
        now_s = self.get_clock().now().nanoseconds * 1.0e-9
        if now_s <= 0.0:
            return
        if self._start_s is None:
            self._start_s = now_s
        elapsed = now_s - self._start_s
        for item in self.events:
            event_id = str(item["fault_id"])
            if event_id in self._published or elapsed < float(item["at_s"]):
                continue
            msg = FaultEvent()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.fault_id = event_id
            msg.target_module = str(item["target_module"])
            msg.action = str(item.get("action", "fail"))
            msg.severity = float(item.get("severity", 1.0))
            msg.duration_s = float(item.get("duration_s", 0.0))
            msg.seed = int(item.get("seed", self.get_parameter("seed").value))
            self.publisher.publish(msg)
            self._published.add(event_id)
            self.get_logger().warning(
                f"Published fault {event_id}: {msg.target_module}/{msg.action}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = XqFaultInjector()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
