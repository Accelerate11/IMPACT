"""P5 0.10 m observed voxel projection and autonomous Frontier selector.

The node consumes only FAST-LIO odometry and the simulated LiDAR.  Gazebo
ground truth is intentionally absent from this process.  One transformed
cloud feeds EGO's 3-D collision map while a persistent 0.10 m observed/free/
occupied projection supplies Frontier extraction and viewpoint selection.
"""

from __future__ import annotations

from collections import deque
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker


def _rotation(q) -> np.ndarray:
    values = np.array((q.w, q.x, q.y, q.z), dtype=np.float64)
    norm = float(np.linalg.norm(values))
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("invalid odometry quaternion")
    w, x, y, z = values / norm
    return np.array(
        (
            (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
            (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
            (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
        ),
        dtype=np.float64,
    )


def _cloud_xyz(message: PointCloud2) -> np.ndarray:
    fields = {field.name: field for field in message.fields}
    if not all(name in fields for name in ("x", "y", "z")):
        return np.empty((0, 3), dtype=np.float32)
    if any(fields[name].datatype != PointField.FLOAT32 for name in ("x", "y", "z")):
        return np.empty((0, 3), dtype=np.float32)
    endian = ">" if message.is_bigendian else "<"
    dtype = np.dtype(
        {
            "names": ("x", "y", "z"),
            "formats": (endian + "f4", endian + "f4", endian + "f4"),
            "offsets": tuple(fields[name].offset for name in ("x", "y", "z")),
            "itemsize": message.point_step,
        }
    )
    count = int(message.width * message.height)
    records = np.frombuffer(message.data, dtype=dtype, count=count)
    points = np.column_stack((records["x"], records["y"], records["z"])).astype(
        np.float32, copy=False
    )
    return points[np.isfinite(points).all(axis=1)]


def _xyz_cloud(points: np.ndarray, stamp, frame_id: str) -> PointCloud2:
    data = np.asarray(points, dtype=np.float32).reshape((-1, 3))
    message = PointCloud2()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.height = 1
    message.width = len(data)
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = 12
    message.row_step = 12 * len(data)
    message.is_dense = True
    message.data = data.tobytes()
    return message


def _bresenham(x0: int, y0: int, x1: int, y1: int):
    dx, sx = abs(x1 - x0), 1 if x0 < x1 else -1
    dy, sy = -abs(y1 - y0), 1 if y0 < y1 else -1
    error = dx + dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            return
        twice = 2 * error
        if twice >= dy:
            error += dy
            x0 += sx
        if twice <= dx:
            error += dx
            y0 += sy


class P5FrontierNode(Node):
    def __init__(self) -> None:
        super().__init__("xq_p5_frontier")
        self.declare_parameter("resolution_m", 0.10)
        self.declare_parameter("map_half_extent_m", 9.5)
        self.declare_parameter("flight_altitude_m", 2.0)
        self.declare_parameter("minimum_mapping_range_m", 0.80)
        self.declare_parameter("clearance_m", 0.65)
        self.declare_parameter("information_radius_m", 2.0)
        self.declare_parameter("distance_lambda", 0.18)
        self.declare_parameter("minimum_cluster_cells", 3)
        self.declare_parameter("goal_tolerance_m", 0.65)
        self.declare_parameter("goal_timeout_s", 55.0)
        self.declare_parameter("finish_empty_cycles", 8)

        self.resolution = float(self.get_parameter("resolution_m").value)
        half = float(self.get_parameter("map_half_extent_m").value)
        self.origin = -half
        self.size = int(math.ceil(2.0 * half / self.resolution))
        self.free = np.zeros((self.size, self.size), dtype=np.bool_)
        self.occupied = np.zeros_like(self.free)
        self.have_odom = False
        self.position = np.zeros(3, dtype=np.float64)
        self.orientation = None
        self.enabled = False
        self.finished = False
        self.scan_count = 0
        self.goal_count = 0
        self.reached_count = 0
        self.failed_count = 0
        self.empty_cycles = 0
        self.active_goal: tuple[float, float, float] | None = None
        self.active_goal_wall = 0.0
        self.started_wall = 0.0
        self.last_scan_stamp = -math.inf
        self.last_clusters: list[list[tuple[int, int]]] = []
        self.last_frontier = np.zeros_like(self.free)
        self.last_reachable_cells = 0
        self.last_viewpoint_mode = "none"

        reliable = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self.cloud_pub = self.create_publisher(PointCloud2, "/xq/p5/cloud_map", reliable)
        self.odom_pub = self.create_publisher(Odometry, "/xq/p5/ego_odom", reliable)
        self.map_pub = self.create_publisher(OccupancyGrid, "/xq/p5/navigation_map", 2)
        self.goal_pub = self.create_publisher(PoseStamped, "/xq/p5/frontier_goal", 10)
        self.marker_pub = self.create_publisher(Marker, "/xq/p5/frontiers", 10)
        self.status_pub = self.create_publisher(String, "/xq/p5/exploration/status", 10)
        self.create_subscription(Odometry, "/localization/odom", self._odom_cb, reliable)
        self.create_subscription(PointCloud2, "/livox/lidar", self._cloud_cb, reliable)
        self.create_subscription(Bool, "/xq/p5/exploration/enable", self._enable_cb, 10)
        self.create_timer(1.0, self._tick)
        self.get_logger().info(
            "P5 Frontier: 0.10 m map, J=I-lambda*d, FAST-LIO+LiDAR only (no ground truth)"
        )

    def _inside(self, ix: int, iy: int) -> bool:
        return 0 <= ix < self.size and 0 <= iy < self.size

    def _index(self, x: float, y: float) -> tuple[int, int]:
        return int(math.floor((x - self.origin) / self.resolution)), int(
            math.floor((y - self.origin) / self.resolution)
        )

    def _world(self, ix: int, iy: int) -> tuple[float, float]:
        return (
            self.origin + (ix + 0.5) * self.resolution,
            self.origin + (iy + 0.5) * self.resolution,
        )

    def _enable_cb(self, message: Bool) -> None:
        if message.data and not self.enabled:
            self.enabled = True
            self.started_wall = time.monotonic()
            self.get_logger().info("Autonomous Frontier exploration enabled")
        elif not message.data:
            self.enabled = False

    def _odom_cb(self, source: Odometry) -> None:
        try:
            rotation = _rotation(source.pose.pose.orientation)
        except ValueError:
            return
        p = source.pose.pose.position
        self.position[:] = (p.x, p.y, p.z)
        self.orientation = source.pose.pose.orientation
        self.have_odom = True

        output = Odometry()
        output.header = source.header
        output.header.frame_id = "xq_lio_map"
        output.child_frame_id = "livox_imu"
        output.pose = source.pose
        body = source.twist.twist.linear
        velocity = rotation @ np.array((body.x, body.y, body.z), dtype=np.float64)
        output.twist = source.twist
        output.twist.twist.linear.x = float(velocity[0])
        output.twist.twist.linear.y = float(velocity[1])
        output.twist.twist.linear.z = float(velocity[2])
        self.odom_pub.publish(output)

    def _cloud_cb(self, message: PointCloud2) -> None:
        if not self.have_odom:
            return
        stamp_s = float(message.header.stamp.sec) + 1e-9 * float(message.header.stamp.nanosec)
        if stamp_s - self.last_scan_stamp < 0.18:
            return
        self.last_scan_stamp = stamp_s
        sensor = _cloud_xyz(message)
        if len(sensor) == 0:
            return
        try:
            rotation = _rotation(self.orientation)
        except ValueError:
            return
        # FAST-LIO's calibrated Mid-360 -> IMU extrinsic from xq_p4.yaml.
        sensor = sensor.astype(np.float64, copy=False)
        sensor += np.array((0.04, 0.0, 0.12), dtype=np.float64)
        mapped = sensor @ rotation.T + self.position
        ranges = np.linalg.norm(sensor, axis=1)
        valid = np.isfinite(mapped).all(axis=1) & (ranges >= 0.35) & (ranges <= 30.0)
        mapped = mapped[valid]
        mapped_ranges = ranges[valid]
        if len(mapped) == 0:
            return

        # Voxelize before feeding EGO.  This is its required 0.10 m navigation
        # map resolution and bounds the planner's cloud callback cost.
        minimum_mapping_range = float(self.get_parameter("minimum_mapping_range_m").value)
        planner_points = mapped[mapped_ranges >= minimum_mapping_range]
        keys = np.floor(planner_points / self.resolution).astype(np.int32)
        _, unique = np.unique(keys, axis=0, return_index=True)
        planner_cloud = planner_points[np.sort(unique)].astype(np.float32)
        self.cloud_pub.publish(_xyz_cloud(planner_cloud, message.header.stamp, "xq_lio_map"))

        # 2-D observed/free/occupied projection at the flight corridor.  The
        # closest return in each angular bin gives deterministic ray casting.
        delta = mapped[:, :2] - self.position[:2]
        radial = np.linalg.norm(delta, axis=1)
        flight_altitude = float(self.get_parameter("flight_altitude_m").value)
        # The Frontier map is a horizontal navigation slice.  Projecting
        # steep floor/ceiling returns into XY creates a false obstacle ring
        # around the vehicle; only geometry intersecting the vehicle's flight
        # slab belongs in this 2-D map.  EGO still receives the complete 3-D
        # cloud above.
        corridor = (
            (mapped[:, 2] >= flight_altitude - 0.45)
            & (mapped[:, 2] <= flight_altitude + 0.45)
            & (radial >= minimum_mapping_range)
            & (radial <= 14.0)
        )
        hits = mapped[corridor]
        radial = radial[corridor]
        if len(hits):
            bins = np.floor((np.arctan2(hits[:, 1] - self.position[1], hits[:, 0] - self.position[0]) + math.pi) * 180.0 / math.pi).astype(np.int32)
            order = np.argsort(radial)
            _, first = np.unique(bins[order], return_index=True)
            hits = hits[order[first]]
            sx, sy = self._index(float(self.position[0]), float(self.position[1]))
            if self._inside(sx, sy):
                self.free[sx, sy] = True
            for hit in hits:
                ex, ey = self._index(float(hit[0]), float(hit[1]))
                if not self._inside(ex, ey) or not self._inside(sx, sy):
                    continue
                cells = list(_bresenham(sx, sy, ex, ey))
                for ix, iy in cells[:-1]:
                    if self._inside(ix, iy) and not self.occupied[ix, iy]:
                        self.free[ix, iy] = True
                self.occupied[ex, ey] = True
                self.free[ex, ey] = False
        self.scan_count += 1

    def _inflated(self) -> np.ndarray:
        cells = int(math.ceil(float(self.get_parameter("clearance_m").value) / self.resolution))
        inflated = self.occupied.copy()
        occupied_indices = np.argwhere(self.occupied)
        for dx in range(-cells, cells + 1):
            for dy in range(-cells, cells + 1):
                if dx * dx + dy * dy > cells * cells:
                    continue
                shifted = occupied_indices + np.array((dx, dy))
                valid = (
                    (shifted[:, 0] >= 0)
                    & (shifted[:, 0] < self.size)
                    & (shifted[:, 1] >= 0)
                    & (shifted[:, 1] < self.size)
                ) if len(shifted) else np.zeros(0, dtype=np.bool_)
                if np.any(valid):
                    inflated[shifted[valid, 0], shifted[valid, 1]] = True
        return inflated

    def _frontier_components(self, frontier: np.ndarray) -> list[list[tuple[int, int]]]:
        remaining = frontier.copy()
        clusters: list[list[tuple[int, int]]] = []
        minimum = int(self.get_parameter("minimum_cluster_cells").value)
        for seed in np.argwhere(frontier):
            sx, sy = int(seed[0]), int(seed[1])
            if not remaining[sx, sy]:
                continue
            queue = deque(((sx, sy),))
            remaining[sx, sy] = False
            cluster = []
            while queue:
                x, y = queue.popleft()
                cluster.append((x, y))
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        nx, ny = x + dx, y + dy
                        if self._inside(nx, ny) and remaining[nx, ny]:
                            remaining[nx, ny] = False
                            queue.append((nx, ny))
            if len(cluster) >= minimum:
                clusters.append(cluster)
        return clusters

    def _reachable_distance(self, traversable: np.ndarray) -> np.ndarray:
        distance = np.full((self.size, self.size), -1, dtype=np.int32)
        sx, sy = self._index(float(self.position[0]), float(self.position[1]))
        if not self._inside(sx, sy):
            return distance
        traversable = traversable.copy()
        traversable[sx, sy] = True
        queue = deque(((sx, sy),))
        distance[sx, sy] = 0
        while queue:
            x, y = queue.popleft()
            # A 360-degree polar scan discretized on a Cartesian grid is often
            # diagonally connected.  Eight-connectivity preserves that real
            # free-space topology; EGO still performs continuous 3-D safety.
            for dx, dy in (
                (1, 0), (-1, 0), (0, 1), (0, -1),
                (1, 1), (1, -1), (-1, 1), (-1, -1),
            ):
                nx, ny = x + dx, y + dy
                if self._inside(nx, ny) and traversable[nx, ny] and distance[nx, ny] < 0:
                    distance[nx, ny] = distance[x, y] + 1
                    queue.append((nx, ny))
        return distance

    def _select(self) -> tuple[tuple[float, float, float] | None, int]:
        unknown = ~(self.free | self.occupied)
        adjacent_unknown = np.zeros_like(unknown)
        adjacent_unknown[1:, :] |= unknown[:-1, :]
        adjacent_unknown[:-1, :] |= unknown[1:, :]
        adjacent_unknown[:, 1:] |= unknown[:, :-1]
        adjacent_unknown[:, :-1] |= unknown[:, 1:]
        inflated = self._inflated()
        traversable = self.free & ~inflated
        frontier = traversable & adjacent_unknown
        self.last_frontier = frontier
        clusters = self._frontier_components(frontier)
        self.last_clusters = clusters
        distance = self._reachable_distance(traversable)
        reachable_indices = np.argwhere(distance >= 0)
        self.last_reachable_cells = int(len(reachable_indices))
        radius = int(round(float(self.get_parameter("information_radius_m").value) / self.resolution))
        lam = float(self.get_parameter("distance_lambda").value)
        best = None
        best_score = -math.inf
        best_mode = "none"
        for cluster in clusters:
            candidates = [cell for cell in cluster if distance[cell] >= 0]
            candidate_mode = "frontier_cell"
            if not candidates:
                # A Frontier is an unknown/free boundary, not necessarily a
                # safe vehicle pose.  Generate viewpoints in already known,
                # reachable free space near the cluster centroid.
                if len(reachable_indices) == 0:
                    continue
                centroid = np.mean(np.asarray(cluster, dtype=np.float64), axis=0)
                squared = np.sum((reachable_indices - centroid) ** 2, axis=1)
                order = np.argsort(squared)
                maximum_view_distance = int(round(2.5 / self.resolution))
                candidates = [
                    (int(reachable_indices[index, 0]), int(reachable_indices[index, 1]))
                    for index in order[:160]
                    if squared[index] <= maximum_view_distance**2
                    and distance[tuple(reachable_indices[index])] * self.resolution >= 0.45
                ]
                candidate_mode = "offset_viewpoint"
                if not candidates:
                    continue
            # Candidate viewpoint is a safe, reachable free Frontier cell.
            for ix, iy in candidates[:: max(1, len(candidates) // 24)]:
                x0, x1 = max(0, ix - radius), min(self.size, ix + radius + 1)
                y0, y1 = max(0, iy - radius), min(self.size, iy + radius + 1)
                xx, yy = np.ogrid[x0:x1, y0:y1]
                disk = (xx - ix) ** 2 + (yy - iy) ** 2 <= radius * radius
                information_m2 = float(np.count_nonzero(unknown[x0:x1, y0:y1] & disk)) * self.resolution**2
                path_m = float(distance[ix, iy]) * self.resolution
                score = information_m2 - lam * path_m
                if score > best_score:
                    best_score = score
                    best = (ix, iy)
                    best_mode = candidate_mode
        if best is None:
            self.last_viewpoint_mode = "none"
            return None, len(clusters)
        self.last_viewpoint_mode = best_mode
        x, y = self._world(*best)
        return (x, y, float(self.get_parameter("flight_altitude_m").value)), len(clusters)

    def _publish_goal(self, goal: tuple[float, float, float]) -> None:
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "xq_lio_map"
        message.pose.position.x, message.pose.position.y, message.pose.position.z = goal
        message.pose.orientation.w = 1.0
        self.goal_pub.publish(message)
        self.active_goal = goal
        self.active_goal_wall = time.monotonic()
        self.goal_count += 1
        self.get_logger().info(f"Frontier goal {self.goal_count}: {goal}")

    def _publish_visuals(self) -> None:
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = "xq_lio_map"
        grid.info.resolution = self.resolution
        grid.info.width = self.size
        grid.info.height = self.size
        grid.info.origin.position.x = self.origin
        grid.info.origin.position.y = self.origin
        grid.info.origin.orientation.w = 1.0
        values = np.full((self.size, self.size), -1, dtype=np.int8)
        values[self.free] = 0
        values[self.occupied] = 100
        # ROS OccupancyGrid is row-major Y then X; our arrays are X,Y.
        grid.data = values.T.reshape(-1).astype(np.int8).tolist()
        self.map_pub.publish(grid)

        marker = Marker()
        marker.header = grid.header
        marker.ns = "p5_frontier"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.10
        marker.scale.y = 0.10
        marker.color.r = 0.05
        marker.color.g = 0.95
        marker.color.b = 1.0
        marker.color.a = 1.0
        for ix, iy in np.argwhere(self.last_frontier):
            x, y = self._world(int(ix), int(iy))
            marker.points.append(Point(x=x, y=y, z=float(self.get_parameter("flight_altitude_m").value)))
        self.marker_pub.publish(marker)

    def _status(self, clusters: int) -> dict[str, object]:
        known = int(np.count_nonzero(self.free | self.occupied))
        return {
            "schema_version": 1,
            "algorithm": "BASELINE_V1",
            "enabled": self.enabled,
            "finished": self.finished,
            "resolution_m": self.resolution,
            "selection_objective": "J=I-lambda*d",
            "distance_lambda": float(self.get_parameter("distance_lambda").value),
            "scan_count": self.scan_count,
            "known_cells": known,
            "known_fraction": known / float(self.size * self.size),
            "frontier_cells": int(np.count_nonzero(self.last_frontier)),
            "frontier_clusters": clusters,
            "reachable_free_cells": self.last_reachable_cells,
            "viewpoint_mode": self.last_viewpoint_mode,
            "goals_published": self.goal_count,
            "goals_reached": self.reached_count,
            "goals_failed": self.failed_count,
            "active_goal": list(self.active_goal) if self.active_goal else None,
            "ground_truth_subscribed": False,
        }

    def _tick(self) -> None:
        clusters = len(self.last_clusters)
        if self.enabled and not self.finished and self.scan_count >= 5:
            if self.active_goal is not None:
                distance = float(np.linalg.norm(self.position - np.asarray(self.active_goal)))
                if distance <= float(self.get_parameter("goal_tolerance_m").value):
                    self.reached_count += 1
                    self.active_goal = None
                    self.empty_cycles = 0
                elif time.monotonic() - self.active_goal_wall > float(self.get_parameter("goal_timeout_s").value):
                    self.failed_count += 1
                    self.active_goal = None
            if self.active_goal is None:
                goal, clusters = self._select()
                if goal is not None:
                    self.empty_cycles = 0
                    self._publish_goal(goal)
                elif clusters == 0 and time.monotonic() - self.started_wall >= 12.0:
                    self.empty_cycles += 1
                    if self.empty_cycles >= int(self.get_parameter("finish_empty_cycles").value):
                        self.finished = True
                        self.get_logger().info("Frontier exhaustion: autonomous exploration finished")
                else:
                    # Frontier still exists: this is not completion.  Keep
                    # rebuilding reachability as new LiDAR rays arrive.
                    self.empty_cycles = 0
        self._publish_visuals()
        message = String()
        message.data = json.dumps(self._status(clusters), separators=(",", ":"))
        self.status_pub.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P5FrontierNode()
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
