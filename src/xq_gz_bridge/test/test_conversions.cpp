#include <array>
#include <cstdint>
#include <string>

#include <gtest/gtest.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <sensor_msgs/msg/point_field.hpp>

#include "xq_gz_bridge/conversions.hpp"

namespace
{
builtin_interfaces::msg::Time fallback_time()
{
  builtin_interfaces::msg::Time result;
  result.sec = 99;
  result.nanosec = 7;
  return result;
}
}  // namespace

TEST(TimeConversion, NormalizesNanoseconds)
{
  gz::msgs::Time input;
  input.set_sec(4);
  input.set_nsec(1000000005);

  const auto result = xq_gz_bridge::to_ros_time(input);
  EXPECT_EQ(result.sec, 5);
  EXPECT_EQ(result.nanosec, 5U);
}

TEST(PointCloudConversion, PreservesPackedPayloadAndOverridesFrame)
{
  gz::msgs::PointCloudPacked input;
  input.mutable_header()->mutable_stamp()->set_sec(12);
  input.set_height(1);
  input.set_width(1);
  input.set_point_step(12);
  input.set_row_step(12);
  input.set_is_dense(true);
  auto * field = input.add_field();
  field->set_name("x");
  field->set_offset(0);
  field->set_datatype(gz::msgs::PointCloudPacked_Field_DataType_FLOAT32);
  field->set_count(1);
  const std::array<uint8_t, 4> bytes{0x00, 0x00, 0x80, 0x3f};
  input.set_data(bytes.data(), bytes.size());

  const auto result = xq_gz_bridge::to_ros_point_cloud(
    input, "xq_mid360_link", fallback_time());

  EXPECT_EQ(result.header.stamp.sec, 12);
  EXPECT_EQ(result.header.frame_id, "xq_mid360_link");
  ASSERT_EQ(result.fields.size(), 1U);
  EXPECT_EQ(result.fields[0].datatype, sensor_msgs::msg::PointField::FLOAT32);
  EXPECT_EQ(result.data.size(), bytes.size());
  EXPECT_TRUE(result.is_dense);
}

TEST(ImuConversion, CopiesMeasurements)
{
  gz::msgs::IMU input;
  input.mutable_orientation()->set_w(0.5);
  input.mutable_angular_velocity()->set_z(1.25);
  input.mutable_linear_acceleration()->set_x(2.5);

  const auto result = xq_gz_bridge::to_ros_imu(input, "imu", fallback_time());

  EXPECT_EQ(result.header.frame_id, "imu");
  EXPECT_DOUBLE_EQ(result.orientation.w, 0.5);
  EXPECT_DOUBLE_EQ(result.angular_velocity.z, 1.25);
  EXPECT_DOUBLE_EQ(result.linear_acceleration.x, 2.5);
  EXPECT_EQ(result.header.stamp.sec, 99);
}

TEST(OdometryConversion, CopiesPoseTwistAndEvaluationFrames)
{
  gz::msgs::Odometry input;
  input.mutable_pose()->mutable_position()->set_x(3.5);
  input.mutable_pose()->mutable_orientation()->set_w(1.0);
  input.mutable_twist()->mutable_linear()->set_y(-0.75);

  const auto result = xq_gz_bridge::to_ros_odometry(
    input, "xq_world", "xq_base_link", fallback_time());

  EXPECT_EQ(result.header.frame_id, "xq_world");
  EXPECT_EQ(result.child_frame_id, "xq_base_link");
  EXPECT_DOUBLE_EQ(result.pose.pose.position.x, 3.5);
  EXPECT_DOUBLE_EQ(result.twist.twist.linear.y, -0.75);
}

TEST(PoseVConversion, SelectsOnlyRequestedModel)
{
  gz::msgs::Pose_V input;
  auto * wrong = input.add_pose();
  wrong->set_name("unrelated_model");
  wrong->mutable_position()->set_x(100.0);
  auto * wanted = input.add_pose();
  wanted->set_name("xq_agent_01");
  wanted->mutable_position()->set_x(2.0);
  wanted->mutable_orientation()->set_w(1.0);

  const auto result = xq_gz_bridge::pose_v_to_ros_odometry(
    input, "xq_agent_01", "xq_world", "xq_base_link", fallback_time());

  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result->pose.pose.position.x, 2.0);
  EXPECT_EQ(result->child_frame_id, "xq_base_link");
}

TEST(TwistConversion, CopiesAllAxes)
{
  geometry_msgs::msg::Twist input;
  input.linear.x = 1.0;
  input.linear.y = 2.0;
  input.linear.z = 3.0;
  input.angular.x = 4.0;
  input.angular.y = 5.0;
  input.angular.z = 6.0;

  const auto result = xq_gz_bridge::to_gz_twist(input);

  EXPECT_DOUBLE_EQ(result.linear().x(), 1.0);
  EXPECT_DOUBLE_EQ(result.linear().y(), 2.0);
  EXPECT_DOUBLE_EQ(result.linear().z(), 3.0);
  EXPECT_DOUBLE_EQ(result.angular().x(), 4.0);
  EXPECT_DOUBLE_EQ(result.angular().y(), 5.0);
  EXPECT_DOUBLE_EQ(result.angular().z(), 6.0);
}
