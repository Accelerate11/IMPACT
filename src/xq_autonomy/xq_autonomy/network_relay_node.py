from __future__ import annotations

import json
from typing import Any, Dict

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from xq_sim_interfaces.msg import FaultEvent

from .network import GroundLinkHeartbeatRelay
from .types import Packet


class XqNetworkRelayNode(Node):
    """Generate and relay deterministic project-local ground-link heartbeats."""

    def __init__(self) -> None:
        super().__init__("xq_network_relay")
        # This node exists only for SIL evidence; never fall back to wall time.
        self.set_parameters([Parameter("use_sim_time", value=True)])
        self.declare_parameter("agent_id", "agent_01")
        self.declare_parameter("seed", 20260820)
        self.declare_parameter("heartbeat_rate_hz", 10.0)
        self.declare_parameter("delay_s", 0.03)
        self.declare_parameter("jitter_s", 0.0)
        self.declare_parameter("max_age_s", 1.0)

        self.agent_id = str(self.get_parameter("agent_id").value)
        self.seed = int(self.get_parameter("seed").value)
        rate_hz = float(self.get_parameter("heartbeat_rate_hz").value)
        if rate_hz <= 0.0:
            raise ValueError("heartbeat_rate_hz must be positive")
        self.model = GroundLinkHeartbeatRelay(
            seed=self.seed,
            delay_s=float(self.get_parameter("delay_s").value),
            jitter_s=float(self.get_parameter("jitter_s").value),
            max_age_s=float(self.get_parameter("max_age_s").value),
        )
        self._seq = 0

        reliable = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        latest = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.tx_pub = self.create_publisher(
            String, "/xq/agent_01/network/tx_heartbeat", reliable
        )
        self.rx_pub = self.create_publisher(
            String, "/xq/agent_01/network/rx_heartbeat", reliable
        )
        self.stats_pub = self.create_publisher(
            String, "/xq/agent_01/network/stats", latest
        )
        self.create_subscription(
            FaultEvent, "/xq/test/fault_event", self._on_fault, reliable
        )
        self.create_timer(1.0 / rate_hz, self._heartbeat_tick)
        self.get_logger().info(
            f"Project-local ground-link relay ready: rate={rate_hz:.1f} Hz, "
            f"seed={self.seed}, healthy drop=0"
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    @staticmethod
    def _json_message(data: Dict[str, Any]) -> String:
        message = String()
        message.data = json.dumps(data, sort_keys=True, separators=(",", ":"))
        return message

    def _on_fault(self, event: FaultEvent) -> None:
        now_s = self._now_s()
        try:
            consumed = self.model.handle_fault(
                target_module=event.target_module,
                action=event.action,
                fault_id=event.fault_id,
                severity=float(event.severity),
                duration_s=float(event.duration_s),
                now_s=now_s,
                seed=int(event.seed) if int(event.seed) != 0 else self.seed,
            )
        except ValueError as error:
            self.get_logger().error(f"Rejected invalid network fault: {error}")
            return
        if not consumed:
            return
        state = self.model.stats_dict(now_s)
        self.get_logger().warning(
            f"Ground link fault update: id={state['fault_id']} "
            f"active={state['active']} drop={state['current_drop_rate']:.3f}"
        )
        self._publish_stats(now_s)

    def _heartbeat_tick(self) -> None:
        now_s = self._now_s()
        if now_s <= 0.0:
            return
        payload = {
            "agent_id": self.agent_id,
            "seq": self._seq,
            "stamp_s": now_s,
        }
        packet = Packet(seq=self._seq, stamp_s=now_s, payload=payload)
        self.tx_pub.publish(self._json_message(payload))
        self.model.send(packet, now_s)
        for delivered in self.model.deliver(now_s):
            rx_payload = dict(delivered.payload)
            rx_payload["delivered_stamp_s"] = now_s
            self.rx_pub.publish(self._json_message(rx_payload))
        self._seq += 1
        self._publish_stats(now_s)

    def _publish_stats(self, now_s: float) -> None:
        stats = self.model.stats_dict(now_s)
        stats["stamp_s"] = now_s
        stats["agent_id"] = self.agent_id
        self.stats_pub.publish(self._json_message(stats))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = XqNetworkRelayNode()
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
