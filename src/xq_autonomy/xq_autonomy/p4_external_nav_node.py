"""Feed FAST-LIO odometry to MAVROS' MAVLink ODOMETRY plugin."""

from __future__ import annotations

import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rosgraph_msgs.msg import Clock as ClockMessage
from std_msgs.msg import String

from xq_autonomy.extnav_clock_guard import ClockGuardPolicy, ExtnavClockGuard
from xq_autonomy.extnav_transport_clock import ExtnavTransportClock
from xq_autonomy.extnav_startup_gate import ExtnavStartupGate
from xq_autonomy.extnav_runtime_graph import BoundedGraphQuery


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
        self.declare_parameter("transport_clock_mode", "raw_ros_clock")
        self.declare_parameter("require_exact_mavros_startup_gate", False)
        self.declare_parameter("require_acquisition_clock_freshness", False)
        self.declare_parameter("acquisition_clock_topic", "/clock")
        self.declare_parameter("clock_ledger_path", "")
        self.transport_clock_mode = str(self.get_parameter("transport_clock_mode").value)
        if self.transport_clock_mode not in {"raw_ros_clock", "epoch_anchor_plus_monotonic"}:
            raise ValueError("unsupported transport_clock_mode")
        self.startup_gate_enabled = bool(self.get_parameter("require_exact_mavros_startup_gate").value)
        self.require_acquisition_clock = bool(self.get_parameter("require_acquisition_clock_freshness").value)
        if self.transport_clock_mode != "raw_ros_clock" and not self.require_acquisition_clock:
            raise ValueError("mapped transport requires independent acquisition-clock freshness checks")
        if self.transport_clock_mode != "raw_ros_clock" and self.get_parameter("use_sim_time").value:
            raise ValueError("mapped transport anchor must use host wall time; acquisition /clock is separate")
        self.transport_mapper = ExtnavTransportClock() if self.transport_clock_mode != "raw_ros_clock" else None
        self.startup_gate = ExtnavStartupGate() if self.startup_gate_enabled else None
        self.graph_query = BoundedGraphQuery(self._query_mavros_endpoints) if self.startup_gate_enabled else None
        self.last_graph_query_result = None
        self.startup_skipped_count = 0
        self.last_acquisition_clock_ns = 0
        self.last_acquisition_clock_monotonic_ns = 0
        self.acquisition_clock_fault = False
        self.input_sequence = 0
        self._active_clock_pair = None
        self._active_ledger = None
        self.ledger_file = None
        ledger_path = str(self.get_parameter("clock_ledger_path").value)
        if ledger_path:
            self.ledger_file = Path(ledger_path).open("x", buffering=1)

        alpha = float(self.get_parameter("velocity_filter_alpha").value)
        if not 0.0 < alpha <= 1.0:
            raise ValueError("velocity_filter_alpha must be in (0, 1]")
        self.alpha = alpha
        self.position_floor = float(self.get_parameter("position_variance_floor").value)
        self.orientation_floor = float(self.get_parameter("orientation_variance_floor").value)
        self.velocity_variance = float(self.get_parameter("velocity_variance").value)
        self.maximum_gap = float(self.get_parameter("maximum_source_gap_s").value)
        # This is a host clock consistency budget, not a calibrated bound on
        # sensor timing or FCU clock error. A violation latches until restart.
        self.clock_guard = ExtnavClockGuard(ClockGuardPolicy(
            max_source_age_ns=min(350_000_000, int(self.maximum_gap * 1_000_000_000)),
            min_rate_hz=max(4.0, float(self.get_parameter("minimum_healthy_rate_hz").value)),
        ))
        self.last_transport_clock_sample = None
        self.clock_sample_uncertain = False

        reliable = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(
            Odometry, str(self.get_parameter("output_topic").value), reliable
        )
        self.status_publisher = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self.ledger_publisher = self.create_publisher(
            String, str(self.get_parameter("status_topic").value) + "/ledger", 1000
        )
        if self.require_acquisition_clock:
            self.create_subscription(ClockMessage,
                str(self.get_parameter("acquisition_clock_topic").value),
                self._acquisition_clock_cb,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(
            Odometry,
            str(self.get_parameter("input_topic").value),
            self._odom_cb,
            reliable,
        )
        self._status_steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.1, self._status_cb, clock=self._status_steady_clock)

        self.previous: tuple[float, tuple[float, float, float]] | None = None
        self.filtered_velocity = (0.0, 0.0, 0.0)
        self.sample_times: list[float] = []
        self.source_count = 0
        self.output_count = 0
        self.rejected_count = 0
        self.rejection_reasons: dict[str, int] = {}
        self.last_rejection_reason = ""
        self.esekf_velocity_count = 0
        self.differenced_velocity_count = 0
        self.last_source_wall = 0.0
        self.last_source_stamp = 0.0
        self.last_source_stamp_ns = 0
        self.get_logger().info(
            "P4 ExternalNav adapter: /localization/odom -> MAVROS ODOMETRY; no ground truth input"
        )

    @staticmethod
    def _covariance_with_floor(
        source: list[float] | tuple[float, ...], position_floor: float, orientation_floor: float
    ) -> list[float]:
        covariance = np.asarray(source, dtype=float)
        if covariance.size != 36:
            raise ValueError("pose_covariance_layout")
        covariance = covariance.reshape(6, 6)
        if not np.isfinite(covariance).all():
            raise ValueError("pose_covariance_nonfinite")
        if np.any(np.diag(covariance) < 0.0):
            raise ValueError("pose_covariance_negative_variance")
        if not np.allclose(covariance, covariance.T, rtol=1.0e-8, atol=1.0e-10):
            raise ValueError("pose_covariance_asymmetric")
        # Average only floating-point asymmetry. Never manufacture a valid
        # covariance by deleting nonfinite entries or its cross-correlations.
        covariance = 0.5 * covariance + 0.5 * covariance.T
        eigenvalues = np.linalg.eigvalsh(covariance)
        if not np.isfinite(eigenvalues).all():
            raise ValueError("pose_covariance_not_psd")
        tolerance = 1.0e-12 * max(1.0, float(np.max(np.abs(eigenvalues))))
        if eigenvalues[0] < -tolerance:
            raise ValueError("pose_covariance_not_psd")
        floors = np.asarray([position_floor] * 3 + [orientation_floor] * 3)
        if not np.isfinite(floors).all() or np.any(floors <= 0.0):
            raise ValueError("invalid_covariance_floor")
        # Adding a nonnegative diagonal preserves PSD. A tiny negative
        # eigenvalue within round-off tolerance gets only a diagonal correction.
        covariance += np.eye(6) * max(0.0, -float(eigenvalues[0]))
        covariance += np.diag(np.maximum(floors - np.diag(covariance), 0.0))
        return covariance.reshape(-1).tolist()

    def _reject(self, reason: str) -> None:
        self.rejected_count += 1
        self.last_rejection_reason = reason
        self.rejection_reasons[reason] = self.rejection_reasons.get(reason, 0) + 1
        self.get_logger().warning(f"ExternalNav rejected source data: {reason}")
        if getattr(self, "_active_ledger", None) is not None:
            self._active_ledger["decision"] = "rejected"
            self._active_ledger["reason"] = reason

    def _odom_cb(self, source: Odometry) -> None:
        # Every source callback keeps raw acquisition, raw ROS/wall-clock and
        # host monotonic observations, including skipped and rejected samples.
        self.input_sequence = getattr(self, "input_sequence", 0) + 1
        self._active_clock_pair = self._sample_transport_clock()
        self._active_ledger = {
            "schema_version": 1, "input_sequence": self.input_sequence,
            "raw_acquisition_stamp_ns": int(source.header.stamp.sec)*1_000_000_000 + int(source.header.stamp.nanosec),
            "acquisition_frame_id": source.header.frame_id,
            "transport_clock_mode": getattr(self, "transport_clock_mode", "raw_ros_clock"),
            "sampling": dict(self.last_transport_clock_sample),
            "decision": "not_forwarded", "reason": "source_callback_incomplete",
            "output_count_before": self.output_count,
        }
        try:
            self._handle_odom(source)
        finally:
            self._active_ledger["output_count_after"] = self.output_count
            self._active_ledger["source_frames_queued_or_replayed"] = False
            text = json.dumps(self._active_ledger, separators=(",", ":"), allow_nan=False)
            if getattr(self, "ledger_publisher", None) is not None:
                self.ledger_publisher.publish(String(data=text))
            if getattr(self, "ledger_file", None) is not None:
                self.ledger_file.write(text + "\n")
            self._active_ledger = None
            self._active_clock_pair = None

    def _handle_odom(self, source: Odometry) -> None:
        stamp_ns = int(source.header.stamp.sec) * 1_000_000_000 + int(source.header.stamp.nanosec)
        if stamp_ns <= 0:
            self._observe_rejected_source_clock(stamp_ns)
            self._reject("source_stamp_nonpositive")
            self._status_cb()
            return
        if stamp_ns <= self.last_source_stamp_ns:
            self._observe_rejected_source_clock(stamp_ns)
            self._reject("source_stamp_not_increasing")
            self._status_cb()
            return
        try:
            q = _normalised_quaternion(source)
            position = source.pose.pose.position
            xyz = (float(position.x), float(position.y), float(position.z))
            stamp = _stamp_s(source)
            if not all(math.isfinite(value) for value in (*xyz, stamp)):
                raise ValueError("non-finite FAST-LIO pose")
            covariance = self._covariance_with_floor(
                source.pose.covariance, self.position_floor, self.orientation_floor
            )
        except (TypeError, ValueError, np.linalg.LinAlgError) as exc:
            self._reject(str(exc))
            return

        acquisition_reason = self._acquisition_freshness_reason(stamp_ns)
        if acquisition_reason:
            self._reject(acquisition_reason)
            self._status_cb()
            return
        if not self._startup_ready():
            self.startup_skipped_count += 1
            self._active_ledger.update(decision="startup_skipped", reason="exact_mavros_endpoint_not_ready")
            return

        # Capture one transport timestamp, then forward it unchanged only if
        # the clock contract holds. Never clamp it or rewrite the source stamp.
        transport, observation_ns = self._active_clock_pair
        if self.clock_sample_uncertain:
            mapper = getattr(self, "transport_mapper", None)
            mapping_fault = mapper is not None and mapper.snapshot()["fault_latched"]
            self._reject("transport_mapping_fault_latched" if mapping_fault else "clock_pair_sampling_uncertain")
            self._status_cb()
            return
        decision = self.clock_guard.observe(stamp_ns, int(transport.nanoseconds), observation_ns)
        if not decision.forward_allowed:
            self._reject("clock_guard:" + ",".join(decision.reasons))
            self._status_cb()  # Publish the revoked local readiness immediately.
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
                    self._reject("velocity_difference_invalid")
                    body_velocity = self.filtered_velocity

        self.previous = (stamp, xyz)
        self.source_count += 1
        self.last_source_wall = time.monotonic()
        self.last_source_stamp = stamp
        self.last_source_stamp_ns = stamp_ns
        self.sample_times.append(self.last_source_wall)
        cutoff = self.last_source_wall - 3.0
        while self.sample_times and self.sample_times[0] < cutoff:
            self.sample_times.pop(0)

        output = Odometry()
        # MAVROS runs on wall time even though the source estimator follows
        # Gazebo /clock. This timestamp is the MAVROS transport clock conversion,
        # not the sensor acquisition time. Status retains the accepted original
        # acquisition stamp; receipt alone never makes repeated source data new.
        output.header.stamp = transport.to_msg()
        output.header.frame_id = "map"
        output.child_frame_id = "base_link"
        output.pose.pose = source.pose.pose
        output.pose.covariance = covariance
        output.twist.twist.linear.x = body_velocity[0]
        output.twist.twist.linear.y = body_velocity[1]
        output.twist.twist.linear.z = body_velocity[2]
        for index in (0, 7, 14, 21, 28, 35):
            output.twist.covariance[index] = self.velocity_variance
        self.publisher.publish(output)
        self.output_count += 1
        self._active_ledger.update(decision="forwarded", reason="accepted",
                                  output_stamp_ns=int(transport.nanoseconds),
                                  output_remote_time_us=int(transport.nanoseconds)//1000)

    def _observe_rejected_source_clock(self, stamp_ns: int) -> None:
        transport, observation_ns = self._active_clock_pair or self._sample_transport_clock()
        if transport is not None:
            self.clock_guard.observe(stamp_ns, int(transport.nanoseconds), observation_ns)

    def _sample_transport_clock(self):
        before = time.monotonic_ns()
        transport = self.get_clock().now()
        after = time.monotonic_ns()
        span = after - before
        self.clock_sample_uncertain = span < 0 or span > 5_000_000
        self.last_transport_clock_sample = {
            "monotonic_before_ns": before, "monotonic_after_ns": after,
            "monotonic_midpoint_ns": (before + after) // 2,
            "transport_stamp_ns": int(transport.nanoseconds),
            "raw_wall_ns": int(transport.nanoseconds),
            "sampling_span_ns": span, "maximum_sampling_span_ns": 5_000_000,
            "sampling_uncertain": self.clock_sample_uncertain,
            "interpretation": "Sampling bracket only; not calibrated wall/FCU clock error",
        }
        mapper = getattr(self, "transport_mapper", None)
        if mapper is not None:
            mapped = mapper.sample(int(transport.nanoseconds), before, after)
            self.last_transport_clock_sample.update(mapper.snapshot()["last_sampling"])
            self.last_transport_clock_sample["transport_stamp_ns"] = mapped.mapped_ns
            self.clock_sample_uncertain = not mapped.accepted
            transport = Time(nanoseconds=mapped.mapped_ns) if mapped.accepted else None
        return transport, (before + after) // 2

    def _acquisition_clock_cb(self, message: ClockMessage) -> None:
        stamp = int(message.clock.sec)*1_000_000_000 + int(message.clock.nanosec)
        if stamp <= 0:
            return
        if stamp < self.last_acquisition_clock_ns:
            self.acquisition_clock_fault = True
        elif stamp > self.last_acquisition_clock_ns:
            self.last_acquisition_clock_ns = stamp
            self.last_acquisition_clock_monotonic_ns = time.monotonic_ns()

    def _acquisition_freshness_reason(self, stamp_ns):
        if not getattr(self, "require_acquisition_clock", False):
            return None
        if self.acquisition_clock_fault:
            return "acquisition_clock_regression_latched"
        age = time.monotonic_ns()-self.last_acquisition_clock_monotonic_ns
        if not self.last_acquisition_clock_ns or not 0 <= age <= int(self.maximum_gap*1e9):
            return "acquisition_clock_missing_or_stale"
        delta = self.last_acquisition_clock_ns-stamp_ns
        if delta < -20_000_000:
            return "source_acquisition_stamp_in_future"
        if delta > int(self.maximum_gap*1e9):
            return "source_acquisition_stamp_stale"
        return None

    def _query_mavros_endpoints(self):
        infos = self.get_subscriptions_info_by_topic(str(self.get_parameter("output_topic").value))
        return [{"node_name": item.node_name, "node_namespace": item.node_namespace,
                 "topic_type": item.topic_type, "endpoint_gid": list(bytes(item.endpoint_gid)),
                 "qos": {"reliability": item.qos_profile.reliability.name,
                         "durability": item.qos_profile.durability.name}} for item in infos]

    def _startup_ready(self):
        if not getattr(self, "startup_gate_enabled", False):
            return True
        now = time.monotonic_ns()
        result = self.graph_query.poll(now)
        if result is not None:
            self.last_graph_query_result = result
            self.startup_gate.observe(result["endpoints"] if result["query_within_budget"] else [],
                                      result["query_finished_monotonic_ns"])
        graph = self.last_graph_query_result
        return bool(graph and graph["query_within_budget"] and
                    0 <= now-graph["query_finished_monotonic_ns"] <= 250_000_000 and
                    self.startup_gate.ready and not self.graph_query.snapshot(now)["pending_exceeds_budget"])

    def destroy_node(self):
        if getattr(self, "graph_query", None) is not None:
            self.graph_query.close()
        if getattr(self, "ledger_file", None) is not None:
            self.ledger_file.close()
        return super().destroy_node()

    def _status_cb(self) -> None:
        clock_status = self.clock_guard.snapshot(time.monotonic_ns())
        if getattr(self, "transport_mapper", None) is not None:
            clock_status.update(transport_clock_domain="host_epoch_anchor_plus_monotonic",
                timestamps_modified=True, modification_scope="transport_timestamp_only_acquisition_unchanged")
        now = time.monotonic()
        source_age = now - self.last_source_wall if self.last_source_wall else math.inf
        span = self.sample_times[-1] - self.sample_times[0] if len(self.sample_times) > 1 else 0.0
        rate = (len(self.sample_times) - 1) / span if span > 0.0 else 0.0
        minimum_rate = float(self.get_parameter("minimum_healthy_rate_hz").value)
        subscribers = self.publisher.get_subscription_count()
        startup_ready = self._startup_ready()
        source_ready = source_age <= self.maximum_gap and rate >= minimum_rate and subscribers > 0 and startup_ready
        clock_ready = clock_status["clock_ready"] and not self.clock_sample_uncertain
        mapper = getattr(self, "transport_mapper", None)
        mapper_status = mapper.snapshot() if mapper is not None else None
        mapping_fault = mapper_status is not None and mapper_status["fault_latched"]
        source_ready = source_ready and self._acquisition_freshness_reason(self.last_source_stamp_ns) is None
        status = {
            "schema_version": 1,
            "healthy": source_ready and clock_ready,
            "local_source_ready": source_ready,
            "local_clock_ready": clock_ready,
            "clock_fault_latched": bool(clock_status["fault_latched"] or mapping_fault or getattr(self, "acquisition_clock_fault", False)),
            "clock_guard": clock_status,
            "transport_clock_sampling": self.last_transport_clock_sample,
            "transport_clock_mode": getattr(self, "transport_clock_mode", "raw_ros_clock"),
            "transport_clock_mapping": mapper_status,
            "exact_mavros_startup_gate_enabled": getattr(self, "startup_gate_enabled", False),
            "exact_mavros_startup_ready": startup_ready,
            "startup_skipped_count": getattr(self, "startup_skipped_count", 0),
            "startup_gate": self.startup_gate.snapshot() if getattr(self, "startup_gate", None) else None,
            "graph_query": self.last_graph_query_result if getattr(self, "startup_gate_enabled", False) else None,
            "acquisition_clock_check_enabled": getattr(self, "require_acquisition_clock", False),
            "last_acquisition_clock_ns": getattr(self, "last_acquisition_clock_ns", None),
            "last_accepted_source_monotonic_ns": clock_status["last_accepted_monotonic_ns"],
            "clock_sample_uncertain": self.clock_sample_uncertain,
            "health_scope": "local_source_and_clock_only_not_FCU_receipt_or_EKF_fusion",
            "clock_ready_is_not_FCU_health": True,
            "source_count": self.source_count,
            "output_count": self.output_count,
            "rejected_count": self.rejected_count,
            "rejection_reasons": dict(self.rejection_reasons),
            "last_rejection_reason": self.last_rejection_reason,
            "esekf_velocity_count": self.esekf_velocity_count,
            "differenced_velocity_count": self.differenced_velocity_count,
            "source_rate_hz": rate,
            "source_age_s": source_age if math.isfinite(source_age) else None,
            "source_age_semantics": "monotonic_since_last_valid_advancing_and_clock_accepted_source",
            "source_stamp_s": self.last_source_stamp,
            "last_accepted_acquisition_stamp_ns": self.last_source_stamp_ns,
            "source_stamp_clock_domain": "estimator_ros_clock",
            "output_stamp_semantics": "mavros_transport_clock_not_acquisition_time",
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
