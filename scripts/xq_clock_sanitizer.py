#!/usr/bin/env python3
"""Republish a recorded simulation clock after removing out-of-order samples."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rosgraph_msgs.msg import Clock


CLOCK_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class MonotonicClockRepublisher(Node):
    def __init__(self) -> None:
        super().__init__("xq_replay_clock_sanitizer")
        self.publisher = self.create_publisher(Clock, "/clock", CLOCK_QOS)
        self.create_subscription(
            Clock, "/xq/recorded_clock", self._callback, CLOCK_QOS
        )
        self.last_stamp_ns: int | None = None
        self.dropped = 0

    def _callback(self, message: Clock) -> None:
        stamp_ns = int(message.clock.sec) * 1_000_000_000 + int(message.clock.nanosec)
        if self.last_stamp_ns is not None and stamp_ns < self.last_stamp_ns:
            self.dropped += 1
            if self.dropped == 1 or self.dropped % 1000 == 0:
                self.get_logger().warn(
                    f"dropped {self.dropped} out-of-order recorded /clock samples"
                )
            return
        self.last_stamp_ns = stamp_ns
        self.publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = MonotonicClockRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
