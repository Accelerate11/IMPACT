#ifndef XQ_GZ_BRIDGE__CONVERSIONS_HPP_
#define XQ_GZ_BRIDGE__CONVERSIONS_HPP_

#include <cstdint>
#include <optional>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <gz/msgs/header.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/time.pb.h>
#include <gz/msgs/twist.pb.h>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace xq_gz_bridge
{

builtin_interfaces::msg::Time to_ros_time(const gz::msgs::Time & time);

builtin_interfaces::msg::Time stamp_from_header(
  const gz::msgs::Header & header,
  const builtin_interfaces::msg::Time & fallback_stamp);

uint8_t to_ros_point_field_type(
  gz::msgs::PointCloudPacked_Field_DataType type);

sensor_msgs::msg::PointCloud2 to_ros_point_cloud(
  const gz::msgs::PointCloudPacked & message,
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & fallback_stamp);

sensor_msgs::msg::Imu to_ros_imu(
  const gz::msgs::IMU & message,
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & fallback_stamp);

nav_msgs::msg::Odometry to_ros_odometry(
  const gz::msgs::Odometry & message,
  const std::string & world_frame_id,
  const std::string & base_frame_id,
  const builtin_interfaces::msg::Time & fallback_stamp);

std::optional<nav_msgs::msg::Odometry> pose_v_to_ros_odometry(
  const gz::msgs::Pose_V & message,
  const std::string & entity_name,
  const std::string & world_frame_id,
  const std::string & base_frame_id,
  const builtin_interfaces::msg::Time & fallback_stamp);

gz::msgs::Twist to_gz_twist(const geometry_msgs::msg::Twist & message);

}  // namespace xq_gz_bridge

#endif  // XQ_GZ_BRIDGE__CONVERSIONS_HPP_
