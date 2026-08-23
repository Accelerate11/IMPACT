"""Deterministic P1 ArduPilot/MAVROS flight-baseline state machine."""

from __future__ import annotations

import json
import math
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, StreamRate
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.executors import ExternalShutdownException


class P1FlightBaseline(Node):
    """Run CONNECT -> GUIDED -> ARM -> TAKEOFF -> HOVER -> LAND."""

    def __init__(self) -> None:
        super().__init__("xq_p1_flight_baseline")
        self.declare_parameter("mavros_prefix", "/uav1/mavros")
        self.declare_parameter("takeoff_altitude_m", 2.0)
        self.declare_parameter("hover_duration_s", 20.0)
        self.declare_parameter("mission_timeout_s", 150.0)
        self.declare_parameter("result_file", "")

        prefix = str(self.get_parameter("mavros_prefix").value).rstrip("/")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(State, f"{prefix}/state", self._state_cb, qos)
        self.create_subscription(
            Odometry, f"{prefix}/local_position/odom", self._odom_cb, qos
        )
        self.arm_client = self.create_client(CommandBool, f"{prefix}/cmd/arming")
        self.mode_client = self.create_client(SetMode, f"{prefix}/set_mode")
        self.takeoff_client = self.create_client(CommandTOL, f"{prefix}/cmd/takeoff")
        self.land_client = self.create_client(CommandTOL, f"{prefix}/cmd/land")
        self.stream_client = self.create_client(
            StreamRate, f"{prefix}/set_stream_rate"
        )

        self.fcu_state = State()
        self.have_state = False
        self.have_odom = False
        self.origin_z = 0.0
        self.current_z = 0.0
        self.phase = "WAIT_FCU"
        self.phase_started = time.monotonic()
        self.started = self.phase_started
        self.last_request = 0.0
        self.pending = None
        self.events: list[dict[str, object]] = []
        self.finalized = False
        self.timer = self.create_timer(0.1, self._tick)
        self._event("START", "P1 state machine created")

    def _state_cb(self, msg: State) -> None:
        self.fcu_state = msg
        self.have_state = True

    def _odom_cb(self, msg: Odometry) -> None:
        self.current_z = float(msg.pose.pose.position.z)
        if not self.have_odom:
            self.origin_z = self.current_z
            self.have_odom = True
            self._event("ODOM_LOCK", f"origin_z={self.origin_z:.3f}")

    def _event(self, kind: str, detail: str) -> None:
        elapsed = time.monotonic() - self.started
        record = {
            "elapsed_s": round(elapsed, 3),
            "kind": kind,
            "phase": self.phase,
            "detail": detail,
        }
        self.events.append(record)
        self.get_logger().info(f"P1 {kind}: {detail}")

    def _transition(self, phase: str, detail: str) -> None:
        self.phase = phase
        self.phase_started = time.monotonic()
        self.pending = None
        self.last_request = 0.0
        self._event("TRANSITION", f"{phase}: {detail}")

    def _send(self, kind: str) -> None:
        if time.monotonic() - self.last_request < 2.0 or self.pending is not None:
            return
        if kind == "mode":
            if not self.mode_client.service_is_ready():
                return
            request = SetMode.Request()
            request.custom_mode = "GUIDED"
            future = self.mode_client.call_async(request)
        elif kind == "arm":
            if not self.arm_client.service_is_ready():
                return
            request = CommandBool.Request()
            request.value = True
            future = self.arm_client.call_async(request)
        elif kind == "takeoff":
            if not self.takeoff_client.service_is_ready():
                return
            request = CommandTOL.Request()
            request.altitude = float(self.get_parameter("takeoff_altitude_m").value)
            future = self.takeoff_client.call_async(request)
        elif kind == "land":
            if not self.land_client.service_is_ready():
                return
            request = CommandTOL.Request()
            request.altitude = 0.0
            future = self.land_client.call_async(request)
        elif kind == "stream":
            if not self.stream_client.service_is_ready():
                return
            request = StreamRate.Request()
            request.stream_id = StreamRate.Request.STREAM_ALL
            request.message_rate = 20
            request.on_off = True
            future = self.stream_client.call_async(request)
        else:
            raise ValueError(kind)
        self.pending = (kind, future)
        self.last_request = time.monotonic()
        self._event("REQUEST", kind)

    def _poll_pending(self) -> None:
        if self.pending is None:
            return
        kind, future = self.pending
        if not future.done():
            return
        self.pending = None
        try:
            response = future.result()
            if kind == "mode":
                accepted = bool(response.mode_sent)
            elif kind == "stream":
                # REQUEST_DATA_STREAM has an empty MAVROS response. Completion
                # of the ROS service transaction is the acknowledgement.
                accepted = True
            else:
                accepted = bool(response.success)
        except Exception as exc:  # rclpy propagates service transport failures here
            self._event("RESPONSE", f"{kind} exception={exc!r}")
            return
        self._event("RESPONSE", f"{kind} accepted={accepted}")
        if not accepted:
            return
        if kind == "stream" and self.phase == "SET_STREAM":
            self._transition("WAIT_ODOM", "MAVLink stream request completed")
        elif kind == "takeoff" and self.phase == "TAKEOFF":
            self._transition("ASCEND", "takeoff accepted")
        elif kind == "land" and self.phase == "LAND":
            self._transition("DESCEND", "land accepted")

    def _relative_altitude(self) -> float:
        return self.current_z - self.origin_z if self.have_odom else math.nan

    def _finish(self, status: str, reason: str) -> None:
        if self.finalized:
            return
        self.finalized = True
        self.phase = "DONE" if status == "PASS" else "FAILED"
        self._event(status, reason)
        result = {
            "schema_version": 1,
            "gate": "P1_ARDUPILOT_MAVROS_FLIGHT_BASELINE",
            "status": status,
            "reason": reason,
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "elapsed_s": round(time.monotonic() - self.started, 3),
            "final": {
                "connected": bool(self.fcu_state.connected),
                "armed": bool(self.fcu_state.armed),
                "mode": self.fcu_state.mode,
                "relative_altitude_m": (
                    self._relative_altitude() if self.have_odom else None
                ),
            },
            "events": self.events,
        }
        result_file = Path(str(self.get_parameter("result_file").value))
        if not result_file:
            self.get_logger().error("result_file parameter is empty")
            return
        result_file.parent.mkdir(parents=True, exist_ok=True)
        temporary = result_file.with_suffix(result_file.suffix + ".tmp")
        temporary.write_text(
            json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        temporary.replace(result_file)

    def _tick(self) -> None:
        if self.finalized:
            return
        self._poll_pending()
        now = time.monotonic()
        if now - self.started > float(self.get_parameter("mission_timeout_s").value):
            self._finish("FAIL", f"mission timeout in {self.phase}")
            return
        if self.have_state and not self.fcu_state.connected and self.phase != "WAIT_FCU":
            self._finish("FAIL", f"FCU disconnected in {self.phase}")
            return

        if self.phase == "WAIT_FCU":
            if self.have_state and self.fcu_state.connected:
                self._transition("SET_STREAM", "FCU heartbeat connected")
        elif self.phase == "SET_STREAM":
            self._send("stream")
        elif self.phase == "WAIT_ODOM":
            if self.have_odom:
                self._transition("SET_GUIDED", "local odometry available")
        elif self.phase == "SET_GUIDED":
            if self.fcu_state.mode == "GUIDED":
                self._transition("ARM", "GUIDED confirmed by state topic")
            else:
                self._send("mode")
        elif self.phase == "ARM":
            if self.fcu_state.armed:
                self._transition("TAKEOFF", "armed confirmed by state topic")
            else:
                self._send("arm")
        elif self.phase == "TAKEOFF":
            self._send("takeoff")
        elif self.phase == "ASCEND":
            target = float(self.get_parameter("takeoff_altitude_m").value)
            if self._relative_altitude() >= 0.8 * target:
                self._transition(
                    "HOVER", f"altitude reached {self._relative_altitude():.3f} m"
                )
            elif now - self.phase_started > 45.0:
                self._finish("FAIL", "takeoff altitude was not reached")
        elif self.phase == "HOVER":
            hover_s = float(self.get_parameter("hover_duration_s").value)
            if now - self.phase_started >= hover_s:
                self._transition("LAND", f"hovered for {hover_s:.1f} s")
        elif self.phase == "LAND":
            self._send("land")
        elif self.phase == "DESCEND":
            if not self.fcu_state.armed and self._relative_altitude() <= 0.30:
                self._finish("PASS", "ARM-TAKEOFF-HOVER-LAND completed")
            elif now - self.phase_started > 60.0:
                self._finish("FAIL", "landing/disarm was not confirmed")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P1FlightBaseline()
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
