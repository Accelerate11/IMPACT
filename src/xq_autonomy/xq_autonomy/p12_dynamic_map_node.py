"""P12 LiDAR-only temporal dynamic voxel mapping node."""

from __future__ import annotations

import json
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String

from .dynamic_planning import path_obstruction
from .dynamic_voxel_map import TemporalDynamicVoxelMap


def _cloud_xyz(message: PointCloud2) -> np.ndarray:
    offsets = {field.name: field.offset for field in message.fields}
    if not all(name in offsets for name in ("x", "y", "z")) or message.point_step <= 0:
        return np.empty((0, 3), dtype=np.float64)
    count = int(message.width) * int(message.height)
    if count <= 0:
        return np.empty((0, 3), dtype=np.float64)
    dtype = np.dtype(
        {
            "names": ("x", "y", "z"),
            "formats": ("<f4", "<f4", "<f4"),
            "offsets": (offsets["x"], offsets["y"], offsets["z"]),
            "itemsize": int(message.point_step),
        }
    )
    values = np.frombuffer(message.data, dtype=dtype, count=count)
    points = np.column_stack((values["x"], values["y"], values["z"])).astype(
        np.float64, copy=False
    )
    return points[np.isfinite(points).all(axis=1)]


def _xyz_cloud(points: np.ndarray, stamp, frame_id: str) -> PointCloud2:
    array = np.asarray(points, dtype=np.float32).reshape((-1, 3))
    message = PointCloud2()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.height = 1
    message.width = len(array)
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = 12
    message.row_step = 12 * len(array)
    message.is_dense = True
    message.data = array.tobytes()
    return message


def _rotation(message: Odometry) -> np.ndarray:
    q = message.pose.pose.orientation
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm <= 1.0e-12:
        return np.eye(3)
    x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
    return np.asarray(
        (
            (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
        ),
        dtype=np.float64,
    )


class P12DynamicMapNode(Node):
    def __init__(self) -> None:
        super().__init__("xq_p12_dynamic_map")
        defaults = {
            "voxel_size_m": 0.25,
            "dynamic_ttl_s": 3.0,
            "static_confirmation_hits": 6,
            "free_confirmation_rays": 3,
            "dynamic_occupied_threshold": 0.35,
            "dynamic_clear_threshold": 0.08,
            "dynamic_confirmation_hits": 3,
            "post_dynamic_static_confirmation_s": 0.0,
            "reversible_static_ttl_s": 0.0,
            "maximum_voxels": 120000,
            "maximum_rays": 900,
            "minimum_range_m": 0.75,
            "maximum_range_m": 12.0,
            "map_frame": "xq_lio_map",
            "path_clearance_radius_m": 0.70,
            "planning_lookahead_m": 4.0,
            "mission_distance_m": 24.0,
            "static_warmup_s": 12.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.map = TemporalDynamicVoxelMap(
            voxel_size_m=self._float("voxel_size_m"),
            dynamic_ttl_s=self._float("dynamic_ttl_s"),
            static_confirmation_hits=self._int("static_confirmation_hits"),
            free_confirmation_rays=self._int("free_confirmation_rays"),
            dynamic_occupied_threshold=self._float("dynamic_occupied_threshold"),
            dynamic_clear_threshold=self._float("dynamic_clear_threshold"),
            dynamic_confirmation_hits=self._int("dynamic_confirmation_hits"),
            post_dynamic_static_confirmation_s=self._float(
                "post_dynamic_static_confirmation_s"
            ),
            reversible_static_ttl_s=self._float("reversible_static_ttl_s"),
            maximum_voxels=self._int("maximum_voxels"),
        )
        self._odom: Odometry | None = None
        self._last_scan_s = -math.inf
        self._last_detection_s: float | None = None
        self._mission_start_x_m: float | None = None
        self._promoted_path_hits = 0
        self._last_forward_blocked_s = -math.inf
        self._raw_path_blocked = False
        self._raw_path_nearest_m = math.inf
        self._first_scan_s: float | None = None
        self._baseline_frozen = False
        self._baseline_static_count = 0
        self._last_sensor_stamp_ns = 0
        self._processing_timing_ms: dict[str, float] = {}
        self._last_status_publish_ms = 0.0
        reliable = QoSProfile(depth=30, reliability=ReliabilityPolicy.RELIABLE)
        latest_sensor = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
        )
        latched = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.dynamic_publisher = self.create_publisher(
            PointCloud2, "/mapping/p12/dynamic_voxels", latched
        )
        self.static_publisher = self.create_publisher(
            PointCloud2, "/mapping/p12/static_voxels", latched
        )
        self.grid_publisher = self.create_publisher(
            OccupancyGrid, "/mapping/p12/occupancy", latched
        )
        self.status_publisher = self.create_publisher(String, "/mapping/p12/status", latched)
        self.create_subscription(Odometry, "/localization/odom", self._odom_cb, reliable)
        self.create_subscription(PointCloud2, "/livox/lidar", self._cloud_cb, latest_sensor)
        self.create_timer(0.20, self._publish)
        self.get_logger().info("P12 dynamic map: LiDAR geometry only; Ground Truth forbidden")

    def _float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _int(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _odom_cb(self, message: Odometry) -> None:
        self._odom = message
        if self._mission_start_x_m is None:
            self._mission_start_x_m = float(message.pose.pose.position.x)

    def _cloud_cb(self, message: PointCloud2) -> None:
        if self._odom is None:
            return
        processing_start_ns = time.perf_counter_ns()
        now_s = self._now_s()
        if now_s <= 0.0 or now_s <= self._last_scan_s:
            return
        if self._first_scan_s is None:
            self._first_scan_s = now_s
        local = _cloud_xyz(message)
        decoded_ns = time.perf_counter_ns()
        if len(local) == 0:
            return
        position = self._odom.pose.pose.position
        origin = np.asarray((position.x, position.y, position.z), dtype=np.float64)
        endpoints = (_rotation(self._odom) @ local.T).T + origin
        transformed_ns = time.perf_counter_ns()
        outcome = self.map.update_scan(
            origin,
            endpoints,
            now_s,
            maximum_rays=self._int("maximum_rays"),
            minimum_range_m=self._float("minimum_range_m"),
            maximum_range_m=self._float("maximum_range_m"),
        )
        map_updated_ns = time.perf_counter_ns()
        assert self._mission_start_x_m is not None
        mission_goal_x = self._mission_start_x_m + self._float("mission_distance_m")
        detection_length = min(
            self._float("maximum_range_m"),
            max(0.0, mission_goal_x - float(origin[0])),
        )
        warmup_complete = bool(
            self._first_scan_s is not None
            and now_s - self._first_scan_s >= self._float("static_warmup_s")
        )
        if warmup_complete and not self._baseline_frozen:
            self._baseline_static_count = self.map.freeze_static_baseline()
            self._baseline_frozen = True
            self.get_logger().info(
                f"P12 baseline static frozen: {self._baseline_static_count} occupied voxels"
            )
        if detection_length > 0.20 and warmup_complete:
            relative = endpoints - origin
            # Apply the certified corridor radius to raw metric points. Voxel
            # inflation here would pull the fixed panels at y=0.86 m into the
            # 0.70 m flight corridor even though the geometry is outside it.
            promotion_radius = self._float("path_clearance_radius_m")
            sweep = (
                (np.linalg.norm(relative, axis=1) >= self._float("minimum_range_m"))
                & (relative[:, 0] >= 0.20)
                & (relative[:, 0] <= detection_length)
                & (np.linalg.norm(relative[:, 1:3], axis=1)
                   <= promotion_radius)
            )
            promotable = self.map.promotable_points(endpoints[sweep])
            path_promotable = promotable[
                promotable[:, 0] - origin[0] <= self._float("planning_lookahead_m")
            ] if len(promotable) else promotable
            self._raw_path_blocked = bool(len(path_promotable))
            self._raw_path_nearest_m = (
                float(np.min(path_promotable[:, 0] - origin[0]))
                if self._raw_path_blocked else math.inf
            )
            self._promoted_path_hits = self.map.promote_dynamic(
                promotable, now_s
            )
        else:
            self._promoted_path_hits = 0
            self._raw_path_blocked = False
            self._raw_path_nearest_m = math.inf
        if outcome["dynamic_hits"] > 0 and self._last_detection_s is None:
            self._last_detection_s = now_s
        self._last_sensor_stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        self._last_scan_s = now_s
        promotion_done_ns = time.perf_counter_ns()
        self._processing_timing_ms = {
            "decode_ms": 1.0e-6 * (decoded_ns - processing_start_ns),
            "transform_ms": 1.0e-6 * (transformed_ns - decoded_ns),
            "update_scan_ms": 1.0e-6 * (map_updated_ns - transformed_ns),
            "voxel_map_stages": self.map.last_update_timing_ms,
            "path_promotion_ms": 1.0e-6 * (promotion_done_ns - map_updated_ns),
            "pre_status_total_ms": 1.0e-6 * (
                promotion_done_ns - processing_start_ns
            ),
            "previous_status_publish_ms": self._last_status_publish_ms,
        }
        # Publish the lightweight safety state immediately after this scan is
        # incorporated. Point clouds and the occupancy grid remain on the 5 Hz
        # visualization timer below; display cadence must not inflate the
        # sensor-to-command latency measured by P13.
        status_start_ns = time.perf_counter_ns()
        self._publish_status(now_s)
        self._last_status_publish_ms = 1.0e-6 * (
            time.perf_counter_ns() - status_start_ns
        )

    def _occupancy_grid(self, dynamic: np.ndarray, static: np.ndarray, stamp) -> OccupancyGrid:
        resolution = self._float("voxel_size_m")
        width = int(math.ceil(30.0 / resolution))
        height = int(math.ceil(6.0 / resolution))
        origin_x, origin_y = -15.0, -3.0
        data = np.full((height, width), -1, dtype=np.int8)
        for points, value in ((static, 100), (dynamic, 80)):
            if not len(points):
                continue
            gx = np.floor((points[:, 0] - origin_x) / resolution).astype(int)
            gy = np.floor((points[:, 1] - origin_y) / resolution).astype(int)
            valid = (gx >= 0) & (gx < width) & (gy >= 0) & (gy < height)
            data[gy[valid], gx[valid]] = value
        message = OccupancyGrid()
        message.header.stamp = stamp
        message.header.frame_id = str(self.get_parameter("map_frame").value)
        message.info.resolution = resolution
        message.info.width = width
        message.info.height = height
        message.info.origin.position.x = origin_x
        message.info.origin.position.y = origin_y
        message.info.origin.orientation.w = 1.0
        message.data = data.reshape(-1).tolist()
        return message

    def _publish_status(self, now_s: float) -> None:
        payload, dynamic = self.map.status_snapshot()
        warmup_complete = bool(
            self._first_scan_s is not None
            and now_s - self._first_scan_s >= self._float("static_warmup_s")
        )
        forward_blocked = False
        instantaneous_blocked = False
        nearest_forward = math.inf
        if self._odom is not None:
            position = self._odom.pose.pose.position
            start = np.asarray((position.x, position.y, position.z), dtype=np.float64)
            mission_goal_x = (
                (self._mission_start_x_m if self._mission_start_x_m is not None else float(start[0]))
                + self._float("mission_distance_m")
            )
            query_length = min(
                self._float("planning_lookahead_m"),
                max(0.21, mission_goal_x - float(start[0])),
            )
            goal = start + np.asarray((query_length, 0.0, 0.0))
            query_points = dynamic[
                np.linalg.norm(dynamic - start, axis=1) >= self._float("minimum_range_m")
            ] if len(dynamic) else dynamic
            instantaneous_blocked, nearest_forward = path_obstruction(
                query_points,
                start,
                goal,
                clearance_radius_m=self._float("path_clearance_radius_m"),
                lookahead_m=query_length,
            )
            if self._raw_path_blocked:
                instantaneous_blocked = True
                nearest_forward = min(nearest_forward, self._raw_path_nearest_m)
            if instantaneous_blocked:
                self._last_forward_blocked_s = now_s
            forward_blocked = bool(
                instantaneous_blocked
                or now_s - self._last_forward_blocked_s <= self._float("dynamic_ttl_s")
            )
        payload.update(
            {
                "phase": "P12_DYNAMIC_MAP",
                "stamp_s": now_s,
                "first_dynamic_detection_s": self._last_detection_s,
                "dynamic_ttl_s": self._float("dynamic_ttl_s"),
                "forward_path_blocked": forward_blocked,
                "forward_path_instantaneously_blocked": instantaneous_blocked,
                "raw_path_blocked": self._raw_path_blocked,
                "raw_path_nearest_range_m": (
                    self._raw_path_nearest_m if math.isfinite(self._raw_path_nearest_m) else None
                ),
                "nearest_forward_dynamic_range_m": (
                    nearest_forward if math.isfinite(nearest_forward) else None
                ),
                "promoted_path_hit_count": self._promoted_path_hits,
                "dynamic_detection_range_m": self._float("maximum_range_m"),
                "planning_lookahead_m": self._float("planning_lookahead_m"),
                "static_warmup_complete": warmup_complete,
                "baseline_static_frozen": self._baseline_frozen,
                "baseline_static_voxel_count": self._baseline_static_count,
                "source_sensor_stamp_ns": self._last_sensor_stamp_ns,
                "processing_timing_ms": self._processing_timing_ms,
                "ground_truth_used": False,
            }
        )
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        self.status_publisher.publish(message)

    def _publish(self) -> None:
        now_s = self._now_s()
        if now_s <= 0.0:
            return
        # A moving viewpoint also reveals previously occluded static surfaces.
        # Keep that generic evidence internal, and expose only the dynamic
        # voxels certified inside the current path sweep to the safety layer.
        dynamic = self.map.path_dynamic_points()
        static = self.map.static_points()
        stamp = self.get_clock().now().to_msg()
        frame = str(self.get_parameter("map_frame").value)
        self.dynamic_publisher.publish(_xyz_cloud(dynamic, stamp, frame))
        self.static_publisher.publish(_xyz_cloud(static, stamp, frame))
        self.grid_publisher.publish(self._occupancy_grid(dynamic, static, stamp))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P12DynamicMapNode()
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
