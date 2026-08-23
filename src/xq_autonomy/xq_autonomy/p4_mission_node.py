"""IMPACT Gate P4: GPS-off ExternalNav square-flight state machine."""

from __future__ import annotations

import json
import math
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, StreamRate
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class P4MissionNode(Node):
    """Verify EKF source, then fly takeoff-hover-square-return-land."""

    # ArduPilot's MAV_CMD_NAV_TAKEOFF handler owns the Guided TakeOff
    # sub-mode until the climb is complete.  Sending SET_POSITION_TARGET while
    # it is climbing switches Guided back to position control and cancels the
    # takeoff altitude target.  Position setpoints therefore start only after
    # ASCEND has confirmed a stable arrival at the requested altitude.
    POSITION_CONTROL_PHASES = frozenset(("HOVER", "TRACK_SQUARE"))

    REQUIRED_PARAMS = {
        "AHRS_EKF_TYPE": 3,
        "EK3_ENABLE": 1,
        "EK3_SRC1_POSXY": 6,
        "EK3_SRC1_VELXY": 6,
        "EK3_SRC1_POSZ": 6,
        "EK3_SRC1_VELZ": 6,
        "EK3_SRC1_YAW": 6,
        "VISO_TYPE": 2,
        "GPS_TYPE": 0,
        "GPS_TYPE2": 0,
        "SIM_GPS_DISABLE": 1,
        "SIM_GPS2_DISABLE": 1,
    }

    def __init__(self) -> None:
        super().__init__("xq_p4_mission")
        self.declare_parameter("mavros_prefix", "/uav1/mavros")
        self.declare_parameter("takeoff_altitude_m", 2.0)
        self.declare_parameter("square_side_m", 2.0)
        self.declare_parameter("hover_duration_s", 5.0)
        self.declare_parameter("arrival_tolerance_m", 0.45)
        self.declare_parameter("mission_timeout_s", 240.0)
        self.declare_parameter("result_file", "")
        self.declare_parameter("extnav_status_topic", "/xq/p4/extnav/status")

        prefix = str(self.get_parameter("mavros_prefix").value).rstrip("/")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=30,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        reliable = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(State, f"{prefix}/state", self._state_cb, qos)
        self.create_subscription(Odometry, f"{prefix}/local_position/odom", self._odom_cb, qos)
        self.create_subscription(
            String,
            str(self.get_parameter("extnav_status_topic").value),
            self._extnav_cb,
            reliable,
        )
        self.setpoint_publisher = self.create_publisher(
            PoseStamped, f"{prefix}/setpoint_position/local", 20
        )
        self.origin_publisher = self.create_publisher(
            GeoPointStamped, f"{prefix}/global_position/set_gp_origin", 10
        )
        self.arm_client = self.create_client(CommandBool, f"{prefix}/cmd/arming")
        self.mode_client = self.create_client(SetMode, f"{prefix}/set_mode")
        self.takeoff_client = self.create_client(CommandTOL, f"{prefix}/cmd/takeoff")
        self.land_client = self.create_client(CommandTOL, f"{prefix}/cmd/land")
        self.stream_client = self.create_client(StreamRate, f"{prefix}/set_stream_rate")
        # MAVROS2 exposes pulled FCU parameters through its standard ROS 2
        # parameter service instead of the deprecated MAVROS ParamGet service.
        self.param_client = self.create_client(
            GetParameters, f"{prefix}/param/get_parameters"
        )

        self.fcu_state = State()
        self.have_state = False
        self.have_odom = False
        self.current_xyz = (0.0, 0.0, 0.0)
        self.current_orientation = None
        self.origin_xyz = (0.0, 0.0, 0.0)
        self.target: tuple[float, float, float] | None = None
        self.extnav_status: dict[str, object] = {}
        self.extnav_last_wall = 0.0
        self.verified_params: dict[str, int] = {}
        self.param_names = list(self.REQUIRED_PARAMS)
        self.param_index = 0
        self.pending_param = None
        self.last_param_request = 0.0
        self.pending_command = None
        self.phase = "WAIT_FCU"
        self.phase_started = time.monotonic()
        self.started = self.phase_started
        self.last_request = 0.0
        self.last_origin_publish = 0.0
        self.arrival_started: float | None = None
        self.waypoints: list[tuple[float, float, float]] = []
        self.waypoint_index = 0
        self.completed_waypoints: list[dict[str, float]] = []
        self.events: list[dict[str, object]] = []
        self.finalized = False
        self.create_timer(0.1, self._tick)
        self._event("START", "P4 GPS-off ExternalNav mission created")

    def _state_cb(self, message: State) -> None:
        self.fcu_state = message
        self.have_state = True

    def _odom_cb(self, message: Odometry) -> None:
        p = message.pose.pose.position
        xyz = (float(p.x), float(p.y), float(p.z))
        if not all(math.isfinite(value) for value in xyz):
            return
        self.current_xyz = xyz
        self.current_orientation = message.pose.pose.orientation
        if not self.have_odom:
            self.origin_xyz = xyz
            self.have_odom = True
            self._event("FCU_ODOM_LOCK", f"origin={xyz}")

    def _extnav_cb(self, message: String) -> None:
        try:
            status = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if isinstance(status, dict):
            self.extnav_status = status
            self.extnav_last_wall = time.monotonic()

    def _event(self, kind: str, detail: str) -> None:
        record = {
            "elapsed_s": round(time.monotonic() - self.started, 3),
            "kind": kind,
            "phase": self.phase,
            "detail": detail,
        }
        self.events.append(record)
        self.get_logger().info(f"P4 {kind}: {detail}")

    def _transition(self, phase: str, detail: str) -> None:
        self.phase = phase
        self.phase_started = time.monotonic()
        self.pending_command = None
        self.last_request = 0.0
        self.arrival_started = None
        self._event("TRANSITION", f"{phase}: {detail}")

    def _publish_origin(self) -> None:
        now = time.monotonic()
        if now - self.last_origin_publish < 1.0:
            return
        message = GeoPointStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.position.latitude = -35.363262
        message.position.longitude = 149.165237
        message.position.altitude = 584.0
        self.origin_publisher.publish(message)
        self.last_origin_publish = now

    def _publish_setpoint(self) -> None:
        if self.target is None:
            return
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.position.x, message.pose.position.y, message.pose.position.z = self.target
        if self.current_orientation is not None:
            message.pose.orientation = self.current_orientation
        else:
            message.pose.orientation.w = 1.0
        self.setpoint_publisher.publish(message)

    def _send_command(self, kind: str) -> None:
        if self.pending_command is not None or time.monotonic() - self.last_request < 2.0:
            return
        if kind == "stream":
            if not self.stream_client.service_is_ready():
                return
            request = StreamRate.Request()
            request.stream_id = StreamRate.Request.STREAM_ALL
            request.message_rate = 20
            request.on_off = True
            future = self.stream_client.call_async(request)
        elif kind == "mode":
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
            future = self.land_client.call_async(request)
        else:
            raise ValueError(kind)
        self.pending_command = (kind, future)
        self.last_request = time.monotonic()
        self._event("REQUEST", kind)

    def _poll_command(self) -> None:
        if self.pending_command is None:
            return
        kind, future = self.pending_command
        if not future.done():
            return
        self.pending_command = None
        try:
            response = future.result()
            accepted = True if kind == "stream" else bool(
                response.mode_sent if kind == "mode" else response.success
            )
        except Exception as exc:
            self._event("RESPONSE", f"{kind} exception={exc!r}")
            return
        self._event("RESPONSE", f"{kind} accepted={accepted}")
        if not accepted:
            return
        if kind == "stream" and self.phase == "SET_STREAM":
            self._transition("VERIFY_NAV", "MAVLink streams requested")
        elif kind == "takeoff" and self.phase == "TAKEOFF":
            self._transition("ASCEND", "takeoff accepted")
        elif kind == "land" and self.phase == "LAND":
            self.target = None
            self._transition("DESCEND", "landing accepted")

    def _poll_param(self) -> None:
        if self.pending_param is not None:
            name, future = self.pending_param
            if not future.done():
                return
            self.pending_param = None
            try:
                response = future.result()
            except Exception as exc:
                self._finish("FAIL", f"parameter {name} query failed: {exc!r}")
                return
            if len(response.values) != 1:
                self._event("PARAM_WAIT", f"{name} not pulled from FCU yet")
                return
            parameter = response.values[0]
            if parameter.type == ParameterType.PARAMETER_INTEGER:
                value = int(parameter.integer_value)
            elif parameter.type == ParameterType.PARAMETER_DOUBLE:
                value = int(round(parameter.double_value))
            elif parameter.type == ParameterType.PARAMETER_NOT_SET:
                self._event("PARAM_WAIT", f"{name} not pulled from FCU yet")
                return
            else:
                self._finish(
                    "FAIL", f"parameter {name} has unexpected ROS type {parameter.type}"
                )
                return
            expected = self.REQUIRED_PARAMS[name]
            if value != expected:
                self._finish("FAIL", f"parameter {name}={value}, expected {expected}")
                return
            self.verified_params[name] = value
            self.param_index += 1
            self._event("PARAM", f"{name}={value}")
        now = time.monotonic()
        if (
            self.param_index < len(self.param_names)
            and self.param_client.service_is_ready()
            and now - self.last_param_request >= 1.0
        ):
            name = self.param_names[self.param_index]
            request = GetParameters.Request()
            request.names = [name]
            self.pending_param = (name, self.param_client.call_async(request))
            self.last_param_request = now

    def _nav_ready(self) -> bool:
        status_fresh = time.monotonic() - self.extnav_last_wall <= 2.0
        return bool(
            self.have_odom
            and status_fresh
            and self.extnav_status.get("healthy") is True
            and int(self.extnav_status.get("mavros_subscribers", 0)) > 0
            and len(self.verified_params) == len(self.REQUIRED_PARAMS)
        )

    def _distance_to_target(self) -> float:
        if self.target is None:
            return math.inf
        return math.sqrt(sum((self.current_xyz[i] - self.target[i]) ** 2 for i in range(3)))

    def _arrived_for(self, dwell_s: float) -> bool:
        tolerance = float(self.get_parameter("arrival_tolerance_m").value)
        if self._distance_to_target() > tolerance:
            self.arrival_started = None
            return False
        if self.arrival_started is None:
            self.arrival_started = time.monotonic()
        return time.monotonic() - self.arrival_started >= dwell_s

    def _finish(self, status: str, reason: str) -> None:
        if self.finalized:
            return
        self.finalized = True
        self.phase = "DONE" if status == "PASS" else "FAILED"
        self._event(status, reason)
        result = {
            "schema_version": 1,
            "gate": "P4_GPS_OFF_EXTERNAL_NAV_CLOSED_LOOP",
            "status": status,
            "reason": reason,
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "elapsed_s": round(time.monotonic() - self.started, 3),
            "verified_parameters": self.verified_params,
            "external_nav": self.extnav_status,
            "completed_waypoints": self.completed_waypoints,
            "checks": {
                "gps_disabled": self.verified_params.get("GPS_TYPE") == 0
                and self.verified_params.get("SIM_GPS_DISABLE") == 1,
                "ekf_sources_external_nav": all(
                    self.verified_params.get(name) == 6
                    for name in ("EK3_SRC1_POSXY", "EK3_SRC1_VELXY", "EK3_SRC1_POSZ", "EK3_SRC1_VELZ", "EK3_SRC1_YAW")
                ),
                "takeoff_and_hover": any(event["phase"] == "HOVER" for event in self.events),
                "square_and_return": len(self.completed_waypoints) == 4,
                "landed_and_disarmed": not bool(self.fcu_state.armed),
            },
            "final": {
                "connected": bool(self.fcu_state.connected),
                "armed": bool(self.fcu_state.armed),
                "mode": self.fcu_state.mode,
                "xyz_m": list(self.current_xyz),
            },
            "events": self.events,
        }
        path = Path(str(self.get_parameter("result_file").value))
        if str(path) in ("", "."):
            self.get_logger().error("result_file parameter is empty")
            return
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        temporary.replace(path)

    def _tick(self) -> None:
        if self.finalized:
            return
        self._publish_origin()
        if self.phase in self.POSITION_CONTROL_PHASES:
            self._publish_setpoint()
        self._poll_command()
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
            self._send_command("stream")
        elif self.phase == "VERIFY_NAV":
            self._poll_param()
            if self.finalized:
                return
            if self._nav_ready():
                self.target = self.current_xyz
                self._transition("SET_GUIDED", "GPS disabled, EKF3 ExternalNav sources and LIO stream verified")
            elif now - self.phase_started > 90.0:
                self._finish("FAIL", "ExternalNav did not become healthy or parameters were not verified")
        elif self.phase == "SET_GUIDED":
            if self.fcu_state.mode == "GUIDED":
                self._transition("ARM", "GUIDED confirmed")
            else:
                self._send_command("mode")
        elif self.phase == "ARM":
            if self.fcu_state.armed:
                self.origin_xyz = self.current_xyz
                altitude = float(self.get_parameter("takeoff_altitude_m").value)
                self.target = (self.origin_xyz[0], self.origin_xyz[1], self.origin_xyz[2] + altitude)
                self._transition("TAKEOFF", "armed with ExternalNav")
            else:
                self._send_command("arm")
        elif self.phase == "TAKEOFF":
            self._send_command("takeoff")
        elif self.phase == "ASCEND":
            if self._arrived_for(2.0):
                self._transition("HOVER", "takeoff altitude reached")
            elif now - self.phase_started > 45.0:
                self._finish("FAIL", "takeoff altitude not reached")
        elif self.phase == "HOVER":
            if now - self.phase_started >= float(self.get_parameter("hover_duration_s").value):
                side = float(self.get_parameter("square_side_m").value)
                x, y, z = self.target
                self.waypoints = [(x + side, y, z), (x + side, y + side, z), (x, y + side, z), (x, y, z)]
                self.waypoint_index = 0
                self.target = self.waypoints[0]
                self._transition("TRACK_SQUARE", "hover complete; first rectangle corner commanded")
        elif self.phase == "TRACK_SQUARE":
            if self._arrived_for(1.5):
                self.completed_waypoints.append(
                    {"index": self.waypoint_index + 1, "x": self.current_xyz[0], "y": self.current_xyz[1], "z": self.current_xyz[2]}
                )
                self.waypoint_index += 1
                if self.waypoint_index >= len(self.waypoints):
                    self._transition("LAND", "rectangle and return completed")
                else:
                    self.target = self.waypoints[self.waypoint_index]
                    self.phase_started = now
                    self.arrival_started = None
                    self._event("WAYPOINT", f"commanded corner {self.waypoint_index + 1}")
            elif now - self.phase_started > 45.0:
                self._finish("FAIL", f"timeout reaching rectangle corner {self.waypoint_index + 1}")
        elif self.phase == "LAND":
            self._send_command("land")
        elif self.phase == "DESCEND":
            if not self.fcu_state.armed and abs(self.current_xyz[2] - self.origin_xyz[2]) <= 0.35:
                self._finish("PASS", "GPS-off LIO ExternalNav takeoff-hover-rectangle-return-land completed")
            elif now - self.phase_started > 60.0:
                self._finish("FAIL", "landing/disarm not confirmed")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P4MissionNode()
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
