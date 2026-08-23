"""P5 autonomous takeoff -> Frontier/EGO exploration -> finish -> land gate."""

from __future__ import annotations

import json
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Bool, String
from traj_utils.msg import Bspline

from .p4_mission_node import P4MissionNode


class P5MissionNode(P4MissionNode):
    """Reuse the verified GPS-off arming contract, replacing square waypoints."""

    POSITION_CONTROL_PHASES = frozenset(("EXPLORE_START",))

    def __init__(self) -> None:
        super().__init__()
        self.declare_parameter("exploration_status_topic", "/xq/p5/exploration/status")
        self.declare_parameter("ego_status_topic", "/xq/p5/ego_adapter/status")
        self.exploration_status: dict[str, object] = {}
        self.ego_status: dict[str, object] = {}
        self.exploration_last_wall = 0.0
        self.ego_last_wall = 0.0
        self.bspline_count = 0
        self.enable_pub = self.create_publisher(Bool, "/xq/p5/exploration/enable", 10)
        self.create_subscription(
            String,
            str(self.get_parameter("exploration_status_topic").value),
            self._exploration_cb,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("ego_status_topic").value),
            self._ego_cb,
            10,
        )
        self.create_subscription(Bspline, "/planning/bspline", self._bspline_cb, 10)
        self._event("P5_READY", "No waypoint list; waiting for autonomous Frontier goals")

    def _exploration_cb(self, message: String) -> None:
        try:
            data = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if isinstance(data, dict):
            self.exploration_status = data
            self.exploration_last_wall = time.monotonic()

    def _ego_cb(self, message: String) -> None:
        try:
            data = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if isinstance(data, dict):
            self.ego_status = data
            self.ego_last_wall = time.monotonic()

    def _bspline_cb(self, _message: Bspline) -> None:
        self.bspline_count += 1

    def _publish_enable(self, value: bool) -> None:
        message = Bool()
        message.data = value
        self.enable_pub.publish(message)

    def _finish(self, status: str, reason: str) -> None:
        if self.finalized:
            return
        self.finalized = True
        self._publish_enable(False)
        self.phase = "DONE" if status == "PASS" else "FAILED"
        self._event(status, reason)
        exploration = self.exploration_status
        adapter = self.ego_status
        result = {
            "schema_version": 1,
            "gate": "P5_BASELINE_MAP_FRONTIER_EGO",
            "baseline": "BASELINE_V1",
            "status": status,
            "reason": reason,
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "elapsed_s": round(time.monotonic() - self.started, 3),
            "verified_parameters": self.verified_params,
            "external_nav": self.extnav_status,
            "exploration": exploration,
            "ego_adapter": adapter,
            "bspline_count": self.bspline_count,
            "checks": {
                "gps_disabled": self.verified_params.get("GPS_TYPE") == 0
                and self.verified_params.get("SIM_GPS_DISABLE") == 1,
                "no_artificial_waypoints": True,
                "frontier_automatic": exploration.get("selection_objective") == "J=I-lambda*d"
                and exploration.get("ground_truth_subscribed") is False,
                "frontier_discovered": int(exploration.get("goals_published", 0)) >= 1,
                "ego_planned": self.bspline_count >= 1,
                "ego_forwarded": int(adapter.get("forwarded", 0)) >= 1,
                "auto_finished": exploration.get("finished") is True,
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
        if self.phase in ("EXPLORE_START", "EXPLORE"):
            self._publish_enable(True)
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
                self._transition("SET_GUIDED", "GPS-off ExternalNav contract verified")
            elif now - self.phase_started > 90.0:
                self._finish("FAIL", "ExternalNav or FCU parameters did not verify")
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
                self.target = self.current_xyz
                self._transition("EXPLORE_START", "takeoff complete; enabling Frontier")
            elif now - self.phase_started > 45.0:
                self._finish("FAIL", "takeoff altitude not reached")
        elif self.phase == "EXPLORE_START":
            fresh = now - self.exploration_last_wall < 2.0 and now - self.ego_last_wall < 2.0
            if fresh and int(self.exploration_status.get("goals_published", 0)) >= 1:
                self._transition("EXPLORE", "first autonomous Frontier goal published")
            elif now - self.phase_started > 45.0:
                self._finish("FAIL", "Frontier did not autonomously produce a goal")
        elif self.phase == "EXPLORE":
            fresh = now - self.exploration_last_wall < 2.0
            if fresh and self.exploration_status.get("finished") is True:
                if self.bspline_count < 1 or int(self.ego_status.get("forwarded", 0)) < 1:
                    self._finish("FAIL", "Frontier finished without a verified EGO trajectory")
                else:
                    self._publish_enable(False)
                    self._transition("LAND", "Frontier exhaustion automatically declared")
            elif now - self.phase_started > 300.0:
                self._finish("FAIL", "autonomous exploration did not finish")
        elif self.phase == "LAND":
            self._publish_enable(False)
            self._send_command("land")
        elif self.phase == "DESCEND":
            self._publish_enable(False)
            if not self.fcu_state.armed and abs(self.current_xyz[2] - self.origin_xyz[2]) <= 0.35:
                checks_ready = (
                    int(self.exploration_status.get("goals_published", 0)) >= 1
                    and self.exploration_status.get("finished") is True
                    and self.bspline_count >= 1
                    and int(self.ego_status.get("forwarded", 0)) >= 1
                )
                self._finish("PASS" if checks_ready else "FAIL", "autonomous P5 exploration and landing completed")
            elif now - self.phase_started > 60.0:
                self._finish("FAIL", "landing/disarm not confirmed")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P5MissionNode()
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
