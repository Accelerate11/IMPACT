#ifndef XQ_GZ_BRIDGE__CAMERA_CONVERSIONS_HPP_
#define XQ_GZ_BRIDGE__CAMERA_CONVERSIONS_HPP_

#include <cstdint>
#include <string>

#include <gz/msgs/camera_info.pb.h>
#include <gz/msgs/image.pb.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

namespace xq_gz_bridge
{
namespace r2
{
constexpr uint32_t kWidth = 320;
constexpr uint32_t kHeight = 240;
constexpr double kHorizontalFov = 1.0471975512;
constexpr double kFocalLength = 277.128129211;
constexpr double kPrincipalX = 159.5;
constexpr double kPrincipalY = 119.5;

// Acquisition stamps are required; no receive-time substitution is permitted.
sensor_msgs::msg::Image to_ros_camera_image(
  const gz::msgs::Image & message, const std::string & optical_frame);
sensor_msgs::msg::CameraInfo to_ros_camera_info(
  const gz::msgs::CameraInfo & message, const std::string & optical_frame);
sensor_msgs::msg::CameraInfo explicit_sdf_camera_info(const std_msgs::msg::Header & header);
bool matches_explicit_sdf_calibration(const sensor_msgs::msg::CameraInfo & info);
}  // namespace r2
}  // namespace xq_gz_bridge

#endif  // XQ_GZ_BRIDGE__CAMERA_CONVERSIONS_HPP_
