#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "xq_gz_bridge/camera_conversions.hpp"

namespace
{
gz::msgs::Image rgb_image(uint32_t row_padding = 0)
{
  gz::msgs::Image image;
  image.mutable_header()->mutable_stamp()->set_sec(12);
  image.mutable_header()->mutable_stamp()->set_nsec(345678901);
  image.set_width(320);
  image.set_height(240);
  image.set_step(320 * 3 + row_padding);
  image.set_pixel_format_type(gz::msgs::RGB_INT8);
  image.set_data(std::string(static_cast<size_t>(image.step()) * image.height(), '\x80'));
  return image;
}

gz::msgs::CameraInfo native_info()
{
  std_msgs::msg::Header header;
  header.stamp.sec = 7;
  header.stamp.nanosec = 123;
  const auto ros = xq_gz_bridge::r2::explicit_sdf_camera_info(header);
  gz::msgs::CameraInfo message;
  message.mutable_header()->mutable_stamp()->set_sec(7);
  message.mutable_header()->mutable_stamp()->set_nsec(123);
  message.set_width(320);
  message.set_height(240);
  for (const auto value : ros.k) {message.mutable_intrinsics()->add_k(value);}
  for (const auto value : ros.p) {message.mutable_projection()->add_p(value);}
  for (const auto value : ros.r) {message.add_rectification_matrix(value);}
  message.mutable_distortion()->set_model(gz::msgs::CameraInfo::Distortion::PLUMB_BOB);
  for (const auto value : ros.d) {message.mutable_distortion()->add_k(value);}
  return message;
}
}  // namespace

TEST(CameraImageConversion, PreservesOriginalPixelsStrideAndAcquisitionStamp)
{
  auto image = rgb_image(4);
  image.mutable_data()->at(0) = '\x01';
  image.mutable_data()->at(1) = '\xff';
  image.mutable_data()->back() = '\x7f';
  const auto converted = xq_gz_bridge::r2::to_ros_camera_image(image, "impact_down_optical");
  EXPECT_EQ(converted.header.stamp.sec, 12);
  EXPECT_EQ(converted.header.stamp.nanosec, 345678901U);
  EXPECT_EQ(converted.header.frame_id, "impact_down_optical");
  EXPECT_EQ(converted.encoding, "rgb8");
  EXPECT_EQ(converted.step, 964U);
  ASSERT_EQ(converted.data.size(), image.data().size());
  EXPECT_EQ(converted.data.front(), 1U);
  EXPECT_EQ(converted.data[1], 255U);
  EXPECT_EQ(converted.data[2], 128U);
  EXPECT_EQ(converted.data.back(), 127U);
}

TEST(CameraImageConversion, MapsEncodingWithoutReorderingSourceBytes)
{
  auto image = rgb_image();
  image.set_pixel_format_type(gz::msgs::BGR_INT8);
  EXPECT_EQ(xq_gz_bridge::r2::to_ros_camera_image(image, "optical").encoding, "bgr8");
  image.set_pixel_format_type(gz::msgs::L_INT8);
  image.set_step(320);
  image.set_data(std::string(320 * 240, '\x47'));
  const auto mono = xq_gz_bridge::r2::to_ros_camera_image(image, "optical");
  EXPECT_EQ(mono.encoding, "mono8");
  EXPECT_EQ(mono.data.front(), 71U);
}

TEST(CameraImageConversion, DropsUnstampedInputsInsteadOfFabricatingReceiveTimes)
{
  auto image = rgb_image();
  image.clear_header();
  EXPECT_THROW(xq_gz_bridge::r2::to_ros_camera_image(image, "optical"), std::invalid_argument);
  image.mutable_header()->mutable_stamp()->set_sec(-1);
  EXPECT_THROW(xq_gz_bridge::r2::to_ros_camera_image(image, "optical"), std::invalid_argument);
  image.mutable_header()->mutable_stamp()->set_sec(1);
  image.mutable_header()->mutable_stamp()->set_nsec(1000000000);
  EXPECT_THROW(xq_gz_bridge::r2::to_ros_camera_image(image, "optical"), std::invalid_argument);
}

TEST(CameraImageConversion, RejectsShortRowsTruncatedPayloadAndWrongDimensions)
{
  auto image = rgb_image();
  image.set_step(959);
  image.set_data(std::string(959 * 240, '\0'));
  EXPECT_THROW(xq_gz_bridge::r2::to_ros_camera_image(image, "optical"), std::invalid_argument);
  image = rgb_image();
  image.mutable_data()->pop_back();
  EXPECT_THROW(xq_gz_bridge::r2::to_ros_camera_image(image, "optical"), std::invalid_argument);
  image = rgb_image();
  image.set_width(640);
  EXPECT_THROW(xq_gz_bridge::r2::to_ros_camera_image(image, "optical"), std::invalid_argument);
  image = rgb_image();
  EXPECT_THROW(xq_gz_bridge::r2::to_ros_camera_image(image, ""), std::invalid_argument);
}

TEST(CameraImageConversion, RejectsUnsupportedPixelFormats)
{
  auto image = rgb_image();
  image.set_pixel_format_type(gz::msgs::L_INT16);
  EXPECT_THROW(xq_gz_bridge::r2::to_ros_camera_image(image, "optical"), std::invalid_argument);
}

TEST(CameraInfoConversion, PreservesNativeCalibrationAndNativeAcquisitionStamp)
{
  const auto native = native_info();
  const auto converted = xq_gz_bridge::r2::to_ros_camera_info(native, "impact_front_optical");
  EXPECT_EQ(converted.header.stamp.sec, 7);
  EXPECT_EQ(converted.header.stamp.nanosec, 123U);
  EXPECT_EQ(converted.header.frame_id, "impact_front_optical");
  EXPECT_DOUBLE_EQ(converted.k[0], 277.128129211);
  EXPECT_DOUBLE_EQ(converted.k[2], 159.5);
  EXPECT_DOUBLE_EQ(converted.k[5], 119.5);
  EXPECT_TRUE(xq_gz_bridge::r2::matches_explicit_sdf_calibration(converted));
}

TEST(CameraInfoConversion, FrozenSdfFallbackHasMatchingImageHeaderAndPinholeGeometry)
{
  const auto image = xq_gz_bridge::r2::to_ros_camera_image(rgb_image(), "impact_down_optical");
  const auto info = xq_gz_bridge::r2::explicit_sdf_camera_info(image.header);
  EXPECT_EQ(info.header, image.header);
  EXPECT_NEAR(info.k[0], 160.0 / std::tan(1.0471975512 / 2.0), 1.0e-6);
  EXPECT_DOUBLE_EQ(info.k[2], 159.5);
  EXPECT_DOUBLE_EQ(info.k[5], 119.5);
  EXPECT_DOUBLE_EQ(info.p[0], info.k[0]);
  EXPECT_DOUBLE_EQ(info.p[2], info.k[2]);
  EXPECT_DOUBLE_EQ(info.p[6], info.k[5]);
}

TEST(CameraInfoConversion, DetectsNativePrincipalPointMismatch)
{
  auto native = native_info();
  native.mutable_intrinsics()->set_k(2, 160.0);
  const auto converted = xq_gz_bridge::r2::to_ros_camera_info(native, "optical");
  EXPECT_FALSE(xq_gz_bridge::r2::matches_explicit_sdf_calibration(converted));
}

TEST(CameraInfoConversion, RejectsIncompleteNonfiniteOrUnsupportedNativeCalibration)
{
  auto native = native_info();
  native.mutable_intrinsics()->clear_k();
  EXPECT_THROW(xq_gz_bridge::r2::to_ros_camera_info(native, "optical"), std::invalid_argument);
  native = native_info();
  native.mutable_projection()->set_p(0, std::numeric_limits<double>::quiet_NaN());
  EXPECT_THROW(xq_gz_bridge::r2::to_ros_camera_info(native, "optical"), std::invalid_argument);
  native = native_info();
  native.mutable_distortion()->set_model(gz::msgs::CameraInfo::Distortion::EQUIDISTANT);
  EXPECT_THROW(xq_gz_bridge::r2::to_ros_camera_info(native, "optical"), std::invalid_argument);
}
