from __future__ import annotations

import math
import time
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as PathMsg
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from xq_sim_interfaces.msg import FaultEvent, HealthStatus, LocalizationQuality, ReplanEvent

from .exploration import OAER2D
from .geometry import wrap_angle
from .localization import DafLioProxy2D
from .mapping import TDSemMap2D
from .planning import R2EgoProxy2D, adaptive_safe_radius, rate_limit_due
from .sentinel import SentinelFSM
from .types import AutonomyMode, HealthLevel, HealthSample, Pose2D


def _stamp_to_float(stamp: TimeMsg) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _float_to_stamp(value: float) -> TimeMsg:
    msg = TimeMsg()
    value = max(0.0, float(value))
    msg.sec = int(value)
    msg.nanosec = int(round((value - msg.sec) * 1.0e9))
    if msg.nanosec >= 1_000_000_000:
        msg.sec += 1
        msg.nanosec -= 1_000_000_000
    return msg


def _yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)


class XqStackNode(Node):
    """Integrated algorithm-level SIL stack.

    Ground truth is intentionally absent from all subscriptions.  The node
    consumes only simulated sensor data and its own command history.
    """

    def __init__(self) -> None:
        super().__init__("xq_stack_node")
        self.declare_parameter("agent_id", "agent_01")
        self.declare_parameter("seed", 20260820)
        self.declare_parameter("map_width_m", 21.0)
        self.declare_parameter("map_height_m", 17.0)
        self.declare_parameter("map_resolution_m", 0.10)
        self.declare_parameter("dynamic_ttl_s", 2.5)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("planning_rate_hz", 2.0)
        self.declare_parameter("health_publish_rate_hz", 2.0)
        self.declare_parameter("dynamic_publish_rate_hz", 5.0)
        self.declare_parameter("planner_deadline_s", 0.8)
        self.declare_parameter("safety_reaction_time_s", 0.20)
        self.declare_parameter("max_speed_mps", 0.8)
        self.declare_parameter("max_yaw_rate_rps", 0.9)
        self.declare_parameter("body_radius_m", 0.25)
        self.declare_parameter("base_margin_m", 0.10)
        self.declare_parameter("lidar_warn_age_s", 0.25)
        self.declare_parameter("lidar_fail_age_s", 0.75)
        self.declare_parameter("initial_x_m", -7.0)
        self.declare_parameter("initial_y_m", -5.0)
        self.declare_parameter("initial_yaw_rad", 0.0)
        self.declare_parameter("max_cloud_points", 360)
        self.declare_parameter("mapping_max_rays", 240)
        self.declare_parameter("planning_warmup_scans", 5)
        self.declare_parameter("obstacle_min_z_m", -0.55)
        self.declare_parameter("obstacle_max_z_m", 0.85)

        self.agent_id = str(self.get_parameter("agent_id").value)
        seed = int(self.get_parameter("seed").value)
        resolution = float(self.get_parameter("map_resolution_m").value)
        self.localizer = DafLioProxy2D(seed=seed)
        self.localizer.reset(
            Pose2D(
                float(self.get_parameter("initial_x_m").value),
                float(self.get_parameter("initial_y_m").value),
                float(self.get_parameter("initial_yaw_rad").value),
            )
        )
        self.mapping = TDSemMap2D(
            width_m=float(self.get_parameter("map_width_m").value),
            height_m=float(self.get_parameter("map_height_m").value),
            resolution_m=resolution,
            dynamic_ttl_s=float(self.get_parameter("dynamic_ttl_s").value),
        )
        self.explorer = OAER2D()
        self.planner = R2EgoProxy2D(
            resolution_m=resolution,
            deadline_s=float(self.get_parameter("planner_deadline_s").value),
        )
        self.sentinel = SentinelFSM()
        self._tf = TransformBroadcaster(self)
        self._path: List[Tuple[float, float]] = []
        self._path_index = 0
        self._last_cmd = Twist()
        self._last_control_s: Optional[float] = None
        self._last_lidar_s: Optional[float] = None
        self._lidar_updates = 0
        self._last_imu_s: Optional[float] = None
        self._last_plan_s = -math.inf
        self._replan_seq = 0
        self._health_seq = 0
        self._last_health_publish_s = -math.inf
        self._last_dynamic_publish_s = -math.inf
        self._published_health_levels: Dict[str, HealthLevel] = {}
        self._last_state: Optional[AutonomyMode] = None
        self._fault_expiry: Dict[str, float] = {}
        self._fault_action: Dict[str, str] = {}
        self._fault_severity: Dict[str, float] = {}
        self._planner_forced_delay_s = 0.0
        self._battery_fraction = 1.0

        lidar_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        reliable_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            PointCloud2,
            "/xq/agent_01/sensors/lidar/points",
            self._on_cloud,
            lidar_qos,
        )
        self.create_subscription(
            Imu,
            "/xq/agent_01/sensors/imu",
            self._on_imu,
            imu_qos,
        )
        self.create_subscription(
            FaultEvent,
            "/xq/test/fault_event",
            self._on_fault,
            reliable_qos,
        )

        self.odom_pub = self.create_publisher(Odometry, "/xq/agent_01/localization/odom", 20)
        self.quality_pub = self.create_publisher(
            LocalizationQuality,
            "/xq/agent_01/localization/quality",
            20,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, "/xq/agent_01/map/nav", 5)
        self.dynamic_pub = self.create_publisher(OccupancyGrid, "/xq/agent_01/map/dynamic", 5)
        self.goal_pub = self.create_publisher(PoseStamped, "/xq/agent_01/exploration/goal", 5)
        self.path_pub = self.create_publisher(PathMsg, "/xq/agent_01/planning/trajectory", 5)
        self.cmd_pub = self.create_publisher(Twist, "/xq/agent_01/cmd_vel", 20)
        self.health_pub = self.create_publisher(HealthStatus, "/xq/agent_01/health", 20)
        self.state_pub = self.create_publisher(String, "/xq/agent_01/autonomy/state", 10)
        self.replan_pub = self.create_publisher(ReplanEvent, "/xq/agent_01/planning/events", 20)

        control_period = 1.0 / float(self.get_parameter("control_rate_hz").value)
        self.create_timer(control_period, self._control_tick)
        self.get_logger().info(
            f"XQ SIL stack ready: agent={self.agent_id}, "
            f"resolution={resolution:.3f} m, truth subscriptions=0"
        )

    def _sim_now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _pointcloud_xy(self, message: PointCloud2) -> np.ndarray:
        fields = {field.name: field for field in message.fields}
        if not all(axis in fields for axis in ("x", "y", "z")) or message.point_step <= 0:
            return np.empty((0, 2), dtype=float)
        x_field, y_field, z_field = fields["x"], fields["y"], fields["z"]
        dtypes = {
            PointField.FLOAT32: "f4",
            PointField.FLOAT64: "f8",
        }
        if any(field.datatype not in dtypes for field in (x_field, y_field, z_field)):
            return np.empty((0, 2), dtype=float)
        endian = ">" if message.is_bigendian else "<"
        count = int(message.width * message.height)
        max_points = max(1, int(self.get_parameter("max_cloud_points").value))
        min_z = float(self.get_parameter("obstacle_min_z_m").value)
        max_z = float(self.get_parameter("obstacle_max_z_m").value)
        if count <= 0:
            return np.empty((0, 2))

        arrays = []
        for field in (x_field, y_field, z_field):
            dtype = np.dtype(endian + dtypes[field.datatype])
            required_bytes = (count - 1) * message.point_step + field.offset + dtype.itemsize
            if required_bytes > len(message.data):
                return np.empty((0, 2))
            arrays.append(
                np.ndarray(
                    shape=(count,),
                    dtype=dtype,
                    buffer=message.data,
                    offset=field.offset,
                    strides=(message.point_step,),
                )
            )
        x, y, z = arrays
        planar_range = np.hypot(x, y)
        valid = (
            np.isfinite(x)
            & np.isfinite(y)
            & np.isfinite(z)
            & (z >= min_z)
            & (z <= max_z)
            & (planar_range >= 0.2)
            & (planar_range <= 20.0)
        )
        valid_indices = np.flatnonzero(valid)
        if valid_indices.size == 0:
            return np.empty((0, 2))
        if valid_indices.size > max_points:
            sample = np.linspace(0, valid_indices.size - 1, max_points, dtype=int)
            valid_indices = valid_indices[sample]
        return np.column_stack((x[valid_indices], y[valid_indices])).astype(float, copy=False)

    def _on_cloud(self, message: PointCloud2) -> None:
        now_s = _stamp_to_float(message.header.stamp)
        if now_s <= 0.0:
            now_s = self._sim_now_s()
        if self._fault_active("lidar", now_s):
            return
        points = self._pointcloud_xy(message)
        if points.shape[0] < 8:
            return
        quality = self.localizer.observe(points)
        self.mapping.update(
            self.localizer.pose,
            points,
            now_s,
            max_rays=int(self.get_parameter("mapping_max_rays").value),
        )
        self._lidar_updates += 1
        self._last_lidar_s = now_s
        self._publish_quality(message.header.stamp, quality)
        self._publish_maps(message.header.stamp, now_s)

    def _on_imu(self, message: Imu) -> None:
        stamp_s = _stamp_to_float(message.header.stamp)
        self._last_imu_s = stamp_s if stamp_s > 0.0 else self._sim_now_s()

    def _on_fault(self, event: FaultEvent) -> None:
        now_s = self._sim_now_s()
        target = str(event.target_module).lower()
        action = str(event.action).lower()
        duration = max(0.0, float(event.duration_s))
        if action in ("clear", "recover"):
            self._fault_expiry.pop(target, None)
            self._recover_fault_target(target)
            return
        self._fault_expiry[target] = now_s + duration if duration > 0.0 else math.inf
        self._fault_action[target] = action
        self._fault_severity[target] = float(event.severity)
        if target == "planner" and action in ("sleep", "timeout"):
            self._planner_forced_delay_s = max(
                float(event.severity),
                float(self.get_parameter("planner_deadline_s").value) + 0.1,
            )
        if target == "battery":
            self._battery_fraction = max(0.0, min(1.0, float(event.severity)))
        if target == "manual":
            self.sentinel.set_manual_override(True)
        self.get_logger().warning(
            f"Injected fault id={event.fault_id} target={target} "
            f"action={action} duration={duration:.2f}s"
        )

    def _recover_fault_target(self, target: str) -> None:
        """Restore mutable proxy state when a scheduled fault ends."""
        self._fault_action.pop(target, None)
        self._fault_severity.pop(target, None)
        if target == "planner":
            self._planner_forced_delay_s = 0.0
        elif target == "battery":
            self._battery_fraction = 1.0

    def _expire_faults(self, now_s: float) -> None:
        for target, expiry in list(self._fault_expiry.items()):
            if now_s > expiry:
                self._fault_expiry.pop(target, None)
                self._recover_fault_target(target)

    def _fault_active(self, target: str, now_s: float) -> bool:
        self._expire_faults(now_s)
        expiry = self._fault_expiry.get(target)
        if expiry is None:
            return False
        return True

    def _control_tick(self) -> None:
        now_s = self._sim_now_s()
        if now_s <= 0.0:
            return
        dt_s = 1.0 / float(self.get_parameter("control_rate_hz").value)
        if self._last_control_s is not None:
            candidate = now_s - self._last_control_s
            if 0.0 < candidate < 0.5:
                dt_s = candidate
        self._last_control_s = now_s
        self.localizer.step(self._last_cmd.linear.x, self._last_cmd.angular.z, dt_s)
        self._update_health(now_s)
        mode = self.sentinel.evaluate()
        self._publish_state(now_s, mode)

        path_blocked = self._path_is_blocked()
        planning_rate_hz = float(self.get_parameter("planning_rate_hz").value)
        plan_due = rate_limit_due(now_s, self._last_plan_s, planning_rate_hz)
        map_initialized = self._lidar_updates >= int(
            self.get_parameter("planning_warmup_scans").value
        )
        if path_blocked and not plan_due:
            # Hold position until the bounded-rate planner is allowed to run.
            self._path = []
        if (
            mode in (AutonomyMode.NORMAL, AutonomyMode.CAUTIOUS, AutonomyMode.RELOCALIZE)
            # F7 is a project-local load proxy: retain localization, mapping,
            # control and safety, but shed the non-critical OAER replanning job.
            and not self.sentinel.essential_only
            and map_initialized
            and plan_due
            and (path_blocked or not self._path)
        ):
            reason = "new_obstacle" if path_blocked else "no_path"
            self._replan(now_s, reason)

        command = self._follow_path(mode)
        self._last_cmd = command
        self.cmd_pub.publish(command)
        self._publish_odom(now_s)
        self._publish_tf(now_s)

    def _update_health(self, now_s: float) -> None:
        self._expire_faults(now_s)
        lidar_age = math.inf if self._last_lidar_s is None else max(0.0, now_s - self._last_lidar_s)
        warn_age = float(self.get_parameter("lidar_warn_age_s").value)
        fail_age = float(self.get_parameter("lidar_fail_age_s").value)
        if lidar_age >= fail_age:
            lidar_level = HealthLevel.FAIL
        elif lidar_age >= warn_age:
            lidar_level = HealthLevel.WARN
        else:
            lidar_level = HealthLevel.OK
        quality = self.localizer.quality
        localization_level = (
            HealthLevel.FAIL
            if self._fault_active("localization", now_s) or quality.degeneracy_score > 0.97
            else HealthLevel.WARN
            if quality.degeneracy_score > 0.80
            else HealthLevel.OK
        )
        planner_level = HealthLevel.FAIL if self._fault_active("planner", now_s) else HealthLevel.OK
        fcu_level = HealthLevel.FAIL if self._fault_active("fcu", now_s) else HealthLevel.OK
        samples = {
            "fcu": HealthSample(fcu_level, 0.0, 1.0 if fcu_level == HealthLevel.OK else 0.0, "SIL link"),
            "lidar": HealthSample(lidar_level, lidar_age, 1.0 if lidar_level == HealthLevel.OK else 0.2, "point cloud age"),
            "localization": HealthSample(
                localization_level,
                0.0,
                1.0 - quality.degeneracy_score,
                "directional observability",
            ),
            "planner": HealthSample(planner_level, 0.0, 1.0 if planner_level == HealthLevel.OK else 0.0, "deadline"),
            "camera": HealthSample(
                HealthLevel.FAIL if self._fault_active("camera", now_s) else HealthLevel.OK,
                0.0,
                0.0 if self._fault_active("camera", now_s) else 1.0,
                "optional semantic sensor",
            ),
            "npu": HealthSample(
                HealthLevel.FAIL if self._fault_active("npu", now_s) else HealthLevel.OK,
                0.0,
                0.0 if self._fault_active("npu", now_s) else 1.0,
                "optional semantic inference",
            ),
            "ground_link": HealthSample(
                HealthLevel.FAIL if self._fault_active("ground_link", now_s) else HealthLevel.OK,
                0.0,
                0.0 if self._fault_active("ground_link", now_s) else 1.0,
                "non-critical link",
            ),
            "cpu": HealthSample(
                HealthLevel.FAIL if self._fault_active("cpu", now_s) else HealthLevel.OK,
                0.0,
                0.0 if self._fault_active("cpu", now_s) else 1.0,
                "project-local load proxy; essential services retained",
            ),
        }
        self.sentinel.set_battery(self._battery_fraction)
        health_rate_hz = float(self.get_parameter("health_publish_rate_hz").value)
        publish_batch = rate_limit_due(now_s, self._last_health_publish_s, health_rate_hz)
        for module, sample in samples.items():
            self.sentinel.set_health(module, sample)
            level_changed = self._published_health_levels.get(module) != sample.level
            if publish_batch or level_changed:
                self._publish_health(now_s, module, sample)
                self._published_health_levels[module] = sample.level
        if publish_batch:
            self._last_health_publish_s = now_s

    def _replan(self, now_s: float, reason: str) -> None:
        self._last_plan_s = now_s
        start = self.mapping.world_to_grid(self.localizer.pose.x, self.localizer.pose.y)
        safe_radius = self._safe_radius()
        occupancy = self.mapping.occupancy_grid()
        reachable = self.planner.reachable_mask(occupancy, start, safe_radius)
        candidate = self.explorer.select_goal(
            self.mapping,
            self.localizer.pose,
            self.localizer.quality.weak_direction,
            reachable_mask=reachable,
        )
        if candidate is None:
            self._path = []
            if not self.mapping.in_bounds(*start):
                outcome = "start_out_of_bounds"
            elif not reachable[start[1], start[0]]:
                outcome = "start_inside_inflated_obstacle"
            elif not self.mapping.frontier_cells():
                outcome = "no_frontier"
            else:
                outcome = "no_reachable_frontier"
            self._publish_replan(now_s, reason, False, True, 0.0, outcome)
            return
        goal = self.mapping.world_to_grid(*candidate.goal)
        wall_started = time.monotonic()
        result = self.planner.plan(
            occupancy,
            start,
            goal,
            safe_radius_m=safe_radius,
            origin_xy=(self.mapping.origin_x, self.mapping.origin_y),
            forced_delay_s=self._planner_forced_delay_s,
        )
        wall_latency = time.monotonic() - wall_started
        if result.accepted:
            self._path = result.path
            self._path_index = 0
            self._publish_goal(now_s, result.path[-1])
            self._publish_path(now_s)
        else:
            self._path = []
        self._publish_replan(now_s, reason, result.accepted, result.brake_fallback, wall_latency, result.reason)

    def _safe_radius(self) -> float:
        return adaptive_safe_radius(
            body_radius_m=float(self.get_parameter("body_radius_m").value),
            base_margin_m=float(self.get_parameter("base_margin_m").value),
            covariance_xy=self.localizer.quality.covariance_xy,
            speed_mps=abs(self._last_cmd.linear.x),
            latency_s=float(self.get_parameter("safety_reaction_time_s").value),
            dynamic_margin_m=0.10 if np.max(self.mapping.dynamic_confidence) > 0.25 else 0.0,
        )

    def _path_is_blocked(self) -> bool:
        if not self._path:
            return False
        grid = self.mapping.occupancy_grid()
        blocked = self.planner.blocked_mask(grid, self._safe_radius())
        for waypoint in self._path[self._path_index : self._path_index + 12]:
            gx, gy = self.mapping.world_to_grid(*waypoint)
            if self.mapping.in_bounds(gx, gy) and blocked[gy, gx]:
                return True
        return False

    def _follow_path(self, mode: AutonomyMode) -> Twist:
        command = Twist()
        if mode in (
            AutonomyMode.BRAKE,
            AutonomyMode.HOVER,
            AutonomyMode.RETURN,
            AutonomyMode.LAND,
            AutonomyMode.MANUAL,
            AutonomyMode.ABORT,
        ):
            return command
        if mode == AutonomyMode.RELOCALIZE:
            command.angular.z = 0.35
            return command
        if not self._path:
            return command
        pose = self.localizer.pose
        while self._path_index < len(self._path):
            wx, wy = self._path[self._path_index]
            if math.hypot(wx - pose.x, wy - pose.y) > 0.20:
                break
            self._path_index += 1
        if self._path_index >= len(self._path):
            self._path = []
            return command
        wx, wy = self._path[self._path_index]
        heading = math.atan2(wy - pose.y, wx - pose.x)
        yaw_error = wrap_angle(heading - pose.yaw)
        max_yaw = float(self.get_parameter("max_yaw_rate_rps").value)
        command.angular.z = max(-max_yaw, min(max_yaw, 1.8 * yaw_error))
        scale = 0.45 if mode == AutonomyMode.CAUTIOUS else 1.0
        alignment = max(0.0, math.cos(yaw_error))
        command.linear.x = float(self.get_parameter("max_speed_mps").value) * scale * alignment
        return command

    def _publish_quality(self, stamp: TimeMsg, quality) -> None:
        msg = LocalizationQuality()
        msg.header.stamp = stamp
        msg.header.frame_id = "xq_odom"
        now_s = _stamp_to_float(stamp)
        if now_s <= 0.0:
            now_s = self._sim_now_s()
        localization_fault = self._fault_active("localization", now_s)
        covariance = np.asarray(quality.covariance_xy, dtype=float).copy()
        if localization_fault:
            injected_variance = max(
                0.04,
                float(self._fault_severity.get("localization", 0.25)),
            )
            covariance[0, 0] = max(float(covariance[0, 0]), injected_variance)
            covariance[1, 1] = max(float(covariance[1, 1]), injected_variance)
        msg.position_covariance = [
            float(covariance[0, 0]), float(covariance[0, 1]), 0.0,
            float(covariance[1, 0]), float(covariance[1, 1]), 0.0,
            0.0, 0.0, 1.0,
        ]
        msg.rotation_covariance = [0.0] * 9
        degeneracy_score = max(
            float(quality.degeneracy_score),
            0.98 if localization_fault else 0.0,
        )
        msg.rotation_covariance[8] = float(0.002 + degeneracy_score * 0.05)
        eig = np.asarray(quality.eigenvalues)
        msg.eig_values_translation = [float(eig[0]), float(eig[-1]), 1.0]
        msg.eig_values_rotation = [1.0, 1.0, 1.0]
        weak = np.asarray(quality.weak_direction)
        msg.weak_direction_map = [float(weak[0]), float(weak[1]), 0.0]
        msg.degeneracy_score = degeneracy_score
        msg.innovation_rms = max(
            float(quality.innovation_rms),
            1.0 if localization_fault else 0.0,
        )
        msg.effective_points = int(quality.effective_points)
        msg.map_match_score = (
            min(float(quality.map_match_score), 0.05)
            if localization_fault
            else float(quality.map_match_score)
        )
        self.quality_pub.publish(msg)

    def _publish_maps(self, stamp: TimeMsg, now_s: float) -> None:
        grid = self.mapping.occupancy_grid()
        msg = OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = "xq_odom"
        msg.info.resolution = self.mapping.resolution_m
        msg.info.width = self.mapping.width
        msg.info.height = self.mapping.height
        msg.info.origin.position.x = self.mapping.origin_x
        msg.info.origin.position.y = self.mapping.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.reshape(-1).astype(int).tolist()
        self.map_pub.publish(msg)

        dynamic_rate_hz = float(self.get_parameter("dynamic_publish_rate_hz").value)
        if not rate_limit_due(now_s, self._last_dynamic_publish_s, dynamic_rate_hz):
            return
        dynamic = OccupancyGrid()
        dynamic.header = msg.header
        dynamic.info = msg.info
        dynamic_data = np.zeros_like(grid, dtype=np.int8)
        dynamic_data[self.mapping.dynamic_confidence > 0.25] = 100
        dynamic.data = dynamic_data.reshape(-1).astype(int).tolist()
        self.dynamic_pub.publish(dynamic)
        self._last_dynamic_publish_s = now_s

    def _publish_odom(self, now_s: float) -> None:
        pose = self.localizer.pose
        msg = Odometry()
        msg.header.stamp = _float_to_stamp(now_s)
        msg.header.frame_id = "xq_odom"
        msg.child_frame_id = "xq_base_link"
        msg.pose.pose.position.x = pose.x
        msg.pose.pose.position.y = pose.y
        qx, qy, qz, qw = _yaw_to_quaternion(pose.yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        covariance = self.localizer.quality.covariance_xy
        msg.pose.covariance[0] = float(covariance[0, 0])
        msg.pose.covariance[1] = float(covariance[0, 1])
        msg.pose.covariance[6] = float(covariance[1, 0])
        msg.pose.covariance[7] = float(covariance[1, 1])
        msg.pose.covariance[35] = 0.02
        msg.twist.twist = self._last_cmd
        self.odom_pub.publish(msg)

    def _publish_tf(self, now_s: float) -> None:
        pose = self.localizer.pose
        transform = TransformStamped()
        transform.header.stamp = _float_to_stamp(now_s)
        transform.header.frame_id = "xq_odom"
        transform.child_frame_id = "xq_base_link"
        transform.transform.translation.x = pose.x
        transform.transform.translation.y = pose.y
        qx, qy, qz, qw = _yaw_to_quaternion(pose.yaw)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self._tf.sendTransform(transform)

    def _publish_goal(self, now_s: float, goal: Tuple[float, float]) -> None:
        msg = PoseStamped()
        msg.header.stamp = _float_to_stamp(now_s)
        msg.header.frame_id = "xq_odom"
        msg.pose.position.x = goal[0]
        msg.pose.position.y = goal[1]
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)

    def _publish_path(self, now_s: float) -> None:
        msg = PathMsg()
        msg.header.stamp = _float_to_stamp(now_s)
        msg.header.frame_id = "xq_odom"
        for x, y in self._path:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def _publish_state(self, now_s: float, mode: AutonomyMode) -> None:
        if mode != self._last_state:
            self.get_logger().info(
                f"Sentinel transition -> {mode.name} ({self.sentinel.last_reason})"
            )
            self._last_state = mode
        msg = String()
        service_modes = []
        if self.sentinel.geometry_only:
            service_modes.append("GEOMETRY_ONLY")
        if self.sentinel.essential_only:
            service_modes.append("ESSENTIAL_ONLY")
        suffix = "".join(f":{item}" for item in service_modes)
        msg.data = f"{mode.name}:{self.sentinel.last_reason}{suffix}"
        self.state_pub.publish(msg)

    def _publish_health(self, now_s: float, module: str, sample: HealthSample) -> None:
        self._health_seq += 1
        msg = HealthStatus()
        msg.header.stamp = _float_to_stamp(now_s)
        msg.module_id = module
        msg.seq = self._health_seq
        msg.state = int(sample.level)
        msg.age_ms = float(sample.age_s * 1000.0) if math.isfinite(sample.age_s) else -1.0
        msg.latency_ms = 0.0
        msg.quality = float(sample.quality)
        msg.error_code = "" if sample.level == HealthLevel.OK else sample.reason
        msg.detail = sample.reason
        self.health_pub.publish(msg)

    def _publish_replan(
        self,
        now_s: float,
        reason: str,
        accepted: bool,
        brake: bool,
        latency_s: float,
        outcome: str,
    ) -> None:
        self._replan_seq += 1
        msg = ReplanEvent()
        msg.header.stamp = _float_to_stamp(now_s)
        msg.seq = self._replan_seq
        msg.trigger_reason = reason
        msg.trigger_stamp = _float_to_stamp(now_s)
        msg.map_ready_stamp = _float_to_stamp(now_s)
        msg.optimizer_start_stamp = _float_to_stamp(now_s)
        msg.candidate_ready_stamp = _float_to_stamp(now_s + latency_s)
        msg.safety_pass_stamp = _float_to_stamp(now_s + latency_s)
        msg.accepted_stamp = _float_to_stamp(now_s + latency_s)
        msg.latency_ms = latency_s * 1000.0
        msg.accepted = accepted
        msg.brake_fallback = brake
        msg.outcome = outcome
        self.replan_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = XqStackNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Humble can surface the private RCLError type instead of
        # ExternalShutdownException when launch invalidates the context while
        # an executor wait-set is being rebuilt.  Suppress only that shutdown
        # race; genuine runtime errors while the context is healthy must fail.
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
