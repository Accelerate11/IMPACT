#include "xq_gz_bridge/conversions.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <string>

#include <sensor_msgs/msg/point_field.hpp>

namespace xq_gz_bridge
{
namespace
{
constexpr int64_t kNanosecondsPerSecond = 1000000000LL;

bool entity_name_matches(const std::string & candidate, const std::string & expected)
{
  if (candidate == expected) {
    return true;
  }

  const std::string scoped_suffix = "::" + expected;
  return candidate.size() > scoped_suffix.size() &&
         candidate.compare(candidate.size() - scoped_suffix.size(), scoped_suffix.size(), scoped_suffix) == 0;
}

void copy_pose(const gz::msgs::Pose & source, geometry_msgs::msg::Pose & destination)
{
  if (source.has_position()) {
    destination.position.x = source.position().x();
    destination.position.y = source.position().y();
    destination.position.z = source.position().z();
  }
  if (source.has_orientation()) {
    destination.orientation.x = source.orientation().x();
    destination.orientation.y = source.orientation().y();
    destination.orientation.z = source.orientation().z();
    destination.orientation.w = source.orientation().w();
  } else {
    destination.orientation.w = 1.0;
  }
}

void copy_twist(const gz::msgs::Twist & source, geometry_msgs::msg::Twist & destination)
{
  if (source.has_linear()) {
    destination.linear.x = source.linear().x();
    destination.linear.y = source.linear().y();
    destination.linear.z = source.linear().z();
  }
  if (source.has_angular()) {
    destination.angular.x = source.angular().x();
    destination.angular.y = source.angular().y();
    destination.angular.z = source.angular().z();
  }
}
}  // namespace

builtin_interfaces::msg::Time to_ros_time(const gz::msgs::Time & time)
{
  int64_t seconds = time.sec();
  int64_t nanoseconds = time.nsec();

  seconds += nanoseconds / kNanosecondsPerSecond;
  nanoseconds %= kNanosecondsPerSecond;
  if (nanoseconds < 0) {
    nanoseconds += kNanosecondsPerSecond;
    --seconds;
  }

  builtin_interfaces::msg::Time result;
  result.sec = static_cast<int32_t>(std::clamp<int64_t>(
      seconds,
      std::numeric_limits<int32_t>::min(),
      std::numeric_limits<int32_t>::max()));
  result.nanosec = static_cast<uint32_t>(nanoseconds);
  return result;
}

builtin_interfaces::msg::Time stamp_from_header(
  const gz::msgs::Header & header,
  const builtin_interfaces::msg::Time & fallback_stamp)
{
  if (header.has_stamp()) {
    return to_ros_time(header.stamp());
  }
  return fallback_stamp;
}

uint8_t to_ros_point_field_type(gz::msgs::PointCloudPacked_Field_DataType type)
{
  using GzType = gz::msgs::PointCloudPacked_Field_DataType;
  switch (type) {
    case GzType::PointCloudPacked_Field_DataType_INT8:
      return sensor_msgs::msg::PointField::INT8;
    case GzType::PointCloudPacked_Field_DataType_UINT8:
      return sensor_msgs::msg::PointField::UINT8;
    case GzType::PointCloudPacked_Field_DataType_INT16:
      return sensor_msgs::msg::PointField::INT16;
    case GzType::PointCloudPacked_Field_DataType_UINT16:
      return sensor_msgs::msg::PointField::UINT16;
    case GzType::PointCloudPacked_Field_DataType_INT32:
      return sensor_msgs::msg::PointField::INT32;
    case GzType::PointCloudPacked_Field_DataType_UINT32:
      return sensor_msgs::msg::PointField::UINT32;
    case GzType::PointCloudPacked_Field_DataType_FLOAT32:
      return sensor_msgs::msg::PointField::FLOAT32;
    case GzType::PointCloudPacked_Field_DataType_FLOAT64:
      return sensor_msgs::msg::PointField::FLOAT64;
    default:
      return sensor_msgs::msg::PointField::UINT8;
  }
}

sensor_msgs::msg::PointCloud2 to_ros_point_cloud(
  const gz::msgs::PointCloudPacked & message,
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & fallback_stamp)
{
  sensor_msgs::msg::PointCloud2 result;
  result.header.stamp = message.has_header() ?
    stamp_from_header(message.header(), fallback_stamp) : fallback_stamp;
  result.header.frame_id = frame_id;
  result.height = message.height();
  result.width = message.width();
  result.is_bigendian = message.is_bigendian();
  result.point_step = message.point_step();
  result.row_step = message.row_step();
  result.is_dense = message.is_dense();

  result.fields.reserve(static_cast<size_t>(message.field_size()));
  for (int index = 0; index < message.field_size(); ++index) {
    const auto & gz_field = message.field(index);
    sensor_msgs::msg::PointField ros_field;
    ros_field.name = gz_field.name();
    ros_field.offset = gz_field.offset();
    ros_field.datatype = to_ros_point_field_type(gz_field.datatype());
    ros_field.count = gz_field.count();
    result.fields.push_back(std::move(ros_field));
  }

  const auto & data = message.data();
  result.data.resize(data.size());
  std::copy(data.begin(), data.end(), result.data.begin());
  return result;
}

sensor_msgs::msg::Imu to_ros_imu(
  const gz::msgs::IMU & message,
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & fallback_stamp)
{
  sensor_msgs::msg::Imu result;
  result.header.stamp = message.has_header() ?
    stamp_from_header(message.header(), fallback_stamp) : fallback_stamp;
  result.header.frame_id = frame_id;

  if (message.has_orientation()) {
    result.orientation.x = message.orientation().x();
    result.orientation.y = message.orientation().y();
    result.orientation.z = message.orientation().z();
    result.orientation.w = message.orientation().w();
  } else {
    result.orientation.w = 1.0;
    result.orientation_covariance[0] = -1.0;
  }

  if (message.has_angular_velocity()) {
    result.angular_velocity.x = message.angular_velocity().x();
    result.angular_velocity.y = message.angular_velocity().y();
    result.angular_velocity.z = message.angular_velocity().z();
  }
  if (message.has_linear_acceleration()) {
    result.linear_acceleration.x = message.linear_acceleration().x();
    result.linear_acceleration.y = message.linear_acceleration().y();
    result.linear_acceleration.z = message.linear_acceleration().z();
  }
  return result;
}

nav_msgs::msg::Odometry to_ros_odometry(
  const gz::msgs::Odometry & message,
  const std::string & world_frame_id,
  const std::string & base_frame_id,
  const builtin_interfaces::msg::Time & fallback_stamp)
{
  nav_msgs::msg::Odometry result;
  result.header.stamp = message.has_header() ?
    stamp_from_header(message.header(), fallback_stamp) : fallback_stamp;
  result.header.frame_id = world_frame_id;
  result.child_frame_id = base_frame_id;
  if (message.has_pose()) {
    copy_pose(message.pose(), result.pose.pose);
  } else {
    result.pose.pose.orientation.w = 1.0;
  }
  if (message.has_twist()) {
    copy_twist(message.twist(), result.twist.twist);
  }
  return result;
}

std::optional<nav_msgs::msg::Odometry> pose_v_to_ros_odometry(
  const gz::msgs::Pose_V & message,
  const std::string & entity_name,
  const std::string & world_frame_id,
  const std::string & base_frame_id,
  const builtin_interfaces::msg::Time & fallback_stamp)
{
  for (int index = 0; index < message.pose_size(); ++index) {
    const auto & pose = message.pose(index);
    if (!entity_name_matches(pose.name(), entity_name)) {
      continue;
    }

    nav_msgs::msg::Odometry result;
    result.header.stamp = message.has_header() ?
      stamp_from_header(message.header(), fallback_stamp) : fallback_stamp;
    result.header.frame_id = world_frame_id;
    result.child_frame_id = base_frame_id;
    copy_pose(pose, result.pose.pose);
    return result;
  }
  return std::nullopt;
}

gz::msgs::Twist to_gz_twist(const geometry_msgs::msg::Twist & message)
{
  gz::msgs::Twist result;
  result.mutable_linear()->set_x(message.linear.x);
  result.mutable_linear()->set_y(message.linear.y);
  result.mutable_linear()->set_z(message.linear.z);
  result.mutable_angular()->set_x(message.angular.x);
  result.mutable_angular()->set_y(message.angular.y);
  result.mutable_angular()->set_z(message.angular.z);
  return result;
}

}  // namespace xq_gz_bridge
