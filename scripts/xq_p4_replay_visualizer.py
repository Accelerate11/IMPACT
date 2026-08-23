#!/usr/bin/env python3
"""Build RViz-friendly P4 paths, vehicle marker, and dynamic TF from the bag."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster
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


class P4ReplayVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("xq_p4_replay_visualizer")
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
        self.marker_publisher = self.create_publisher(
            Marker, "/xq/replay/uav_marker", 10
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
            "/xq/eval/p4/ground_truth",
            self._truth_callback,
            qos_profile_sensor_data,
        )
        self.lio_origin: tuple[float, float, float, float] | None = None
        self.truth_origin: tuple[float, float, float, float] | None = None
        self.get_logger().info(
            "P4 replay visualizer ready: point cloud TF, LIO path, aligned truth path"
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
        marker.ns = "xq_p4_replay"
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


def main() -> None:
    rclpy.init()
    node = P4ReplayVisualizer()
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
