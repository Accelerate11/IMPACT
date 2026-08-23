#!/usr/bin/env python3
"""Build RViz-friendly P5 paths, EGO trajectory, vehicle marker, and TF."""

from __future__ import annotations

import math
from bisect import bisect_right

import rclpy
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster
from traj_utils.msg import Bspline
from visualization_msgs.msg import Marker


def _yaw(orientation) -> float:
    sin_yaw = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cos_yaw = 1.0 - 2.0 * (
        orientation.y * orientation.y + orientation.z * orientation.z
    )
    return math.atan2(sin_yaw, cos_yaw)


def _yaw_quaternion(yaw: float):
    from geometry_msgs.msg import Quaternion

    result = Quaternion()
    result.z = math.sin(0.5 * yaw)
    result.w = math.cos(0.5 * yaw)
    return result


class P5ReplayVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("xq_p5_replay_visualizer")
        self.frame_id = "xq_lio_map"
        self.lio_path = Path()
        self.lio_path.header.frame_id = self.frame_id
        self.truth_path = Path()
        self.truth_path.header.frame_id = self.frame_id
        self.lio_path_publisher = self.create_publisher(
            Path, "/xq/replay/lio_path", 10
        )
        self.truth_path_publisher = self.create_publisher(
            Path, "/xq/replay/truth_path", 10
        )
        self.ego_path_publisher = self.create_publisher(
            Path, "/xq/replay/ego_path", 10
        )
        self.marker_publisher = self.create_publisher(
            Marker, "/xq/replay/uav_marker", 10
        )
        self.map_marker_publisher = self.create_publisher(
            Marker, "/xq/replay/navigation_obstacles", 10
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry,
            "/localization/odom",
            self._lio_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            "/xq/eval/p5/ground_truth",
            self._truth_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(Bspline, "/planning/bspline", self._bspline_callback, 10)
        self.create_subscription(
            OccupancyGrid, "/xq/p5/navigation_map", self._map_callback, 10
        )
        self.lio_origin: tuple[float, float, float, float] | None = None
        self.truth_origin: tuple[float, float, float, float] | None = None
        self.get_logger().info(
            "P5 replay ready: map/frontiers, LIO/truth paths, EGO B-spline and UAV"
        )

    def _lio_callback(self, message: Odometry) -> None:
        position = message.pose.pose.position
        if self.lio_origin is None:
            self.lio_origin = (
                float(position.x),
                float(position.y),
                float(position.z),
                _yaw(message.pose.pose.orientation),
            )

        pose = PoseStamped()
        pose.header = message.header
        pose.header.frame_id = self.frame_id
        pose.pose = message.pose.pose
        self.lio_path.header.stamp = message.header.stamp
        self.lio_path.poses.append(pose)
        self.lio_path_publisher.publish(self.lio_path)

        transform = TransformStamped()
        transform.header = message.header
        transform.header.frame_id = self.frame_id
        transform.child_frame_id = "livox_imu"
        transform.transform.translation.x = position.x
        transform.transform.translation.y = position.y
        transform.transform.translation.z = position.z
        transform.transform.rotation = message.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

        marker = Marker()
        marker.header = pose.header
        marker.ns = "xq_p5_replay"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = 0.70
        marker.scale.y = 0.16
        marker.scale.z = 0.16
        marker.color.r = 0.10
        marker.color.g = 0.95
        marker.color.b = 0.25
        marker.color.a = 1.0
        self.marker_publisher.publish(marker)

    def _truth_callback(self, message: Odometry) -> None:
        position = message.pose.pose.position
        truth_yaw = _yaw(message.pose.pose.orientation)
        if self.truth_origin is None:
            self.truth_origin = (
                float(position.x),
                float(position.y),
                float(position.z),
                truth_yaw,
            )
        if self.lio_origin is None:
            return

        truth_x, truth_y, truth_z, truth_origin_yaw = self.truth_origin
        lio_x, lio_y, lio_z, lio_origin_yaw = self.lio_origin
        yaw_offset = lio_origin_yaw - truth_origin_yaw
        delta_x = float(position.x) - truth_x
        delta_y = float(position.y) - truth_y
        cosine = math.cos(yaw_offset)
        sine = math.sin(yaw_offset)

        pose = PoseStamped()
        pose.header = message.header
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = lio_x + cosine * delta_x - sine * delta_y
        pose.pose.position.y = lio_y + sine * delta_x + cosine * delta_y
        pose.pose.position.z = lio_z + float(position.z) - truth_z
        pose.pose.orientation = _yaw_quaternion(truth_yaw + yaw_offset)
        self.truth_path.header.stamp = message.header.stamp
        self.truth_path.poses.append(pose)
        self.truth_path_publisher.publish(self.truth_path)

    @staticmethod
    def _de_boor(points, knots, degree: int, value: float):
        """Evaluate the non-uniform B-spline encoded by traj_utils/Bspline."""
        last = len(points) - 1
        span = min(max(bisect_right(knots, value) - 1, degree), last)
        work = [list(points[index + span - degree]) for index in range(degree + 1)]
        for level in range(1, degree + 1):
            for index in range(degree, level - 1, -1):
                knot_index = index + span - degree
                denominator = knots[knot_index + degree - level + 1] - knots[knot_index]
                alpha = 0.0 if abs(denominator) < 1e-12 else (value - knots[knot_index]) / denominator
                work[index] = [
                    (1.0 - alpha) * work[index - 1][axis] + alpha * work[index][axis]
                    for axis in range(3)
                ]
        return work[degree]

    def _bspline_callback(self, message: Bspline) -> None:
        degree = int(message.order)
        points = [(point.x, point.y, point.z) for point in message.pos_pts]
        knots = list(message.knots)
        if degree < 1 or len(points) <= degree or len(knots) < len(points) + degree + 1:
            return
        start = knots[degree]
        stop = knots[len(points)]
        if stop <= start:
            return
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        sample_count = max(30, int(math.ceil((stop - start) / 0.05)) + 1)
        for sample in range(sample_count):
            value = start + (stop - start) * sample / (sample_count - 1)
            xyz = self._de_boor(points, knots, degree, value)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = xyz
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.ego_path_publisher.publish(path)

    def _map_callback(self, message: OccupancyGrid) -> None:
        """Render occupied cells without RViz's Mesa-sensitive Map shader."""
        marker = Marker()
        marker.header = message.header
        marker.ns = "xq_p5_navigation_obstacles"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(message.info.resolution)
        marker.scale.y = float(message.info.resolution)
        marker.scale.z = 0.22
        marker.color.r = 0.92
        marker.color.g = 0.25
        marker.color.b = 0.12
        marker.color.a = 0.92
        origin = message.info.origin.position
        width = int(message.info.width)
        resolution = float(message.info.resolution)
        for index, occupancy in enumerate(message.data):
            if occupancy < 50:
                continue
            x_index = index % width
            y_index = index // width
            marker.points.append(
                Point(
                    x=origin.x + (x_index + 0.5) * resolution,
                    y=origin.y + (y_index + 0.5) * resolution,
                    z=0.11,
                )
            )
        self.map_marker_publisher.publish(marker)


def main() -> None:
    rclpy.init()
    node = P5ReplayVisualizer()
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
