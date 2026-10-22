#include "xq_gz_bridge/camera_conversions.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace xq_gz_bridge
{
namespace r2
{
namespace
{
builtin_interfaces::msg::Time acquisition_stamp(const gz::msgs::Header & header)
{
  if (!header.has_stamp() || header.stamp().sec() < 0 ||
    header.stamp().sec() > std::numeric_limits<int32_t>::max() ||
    header.stamp().nsec() < 0 || header.stamp().nsec() >= 1000000000)
  {
    throw std::invalid_argument("Camera acquisition stamp is missing or out of range");
  }
  builtin_interfaces::msg::Time result;
  result.sec = static_cast<int32_t>(header.stamp().sec());
  result.nanosec = static_cast<uint32_t>(header.stamp().nsec());
  return result;
}

void validate_shape(uint32_t width, uint32_t height, const std::string & frame)
{
  if (width != kWidth || height != kHeight || frame.empty()) {
    throw std::invalid_argument("Camera dimensions/frame do not match the fixed R2 model");
  }
}
}  // namespace

sensor_msgs::msg::Image to_ros_camera_image(
  const gz::msgs::Image & message, const std::string & optical_frame)
{
  validate_shape(message.width(), message.height(), optical_frame);
  if (!message.has_header()) {
    throw std::invalid_argument("Camera image has no acquisition header");
  }
  sensor_msgs::msg::Image result;
  result.header.stamp = acquisition_stamp(message.header());
  result.header.frame_id = optical_frame;
  result.width = message.width();
  result.height = message.height();
  uint32_t channels = 0;
  switch (message.pixel_format_type()) {
    case gz::msgs::RGB_INT8:
      channels = 3;
      result.encoding = "rgb8";
      break;
    case gz::msgs::BGR_INT8:
      channels = 3;
      result.encoding = "bgr8";
      break;
    case gz::msgs::L_INT8:
      channels = 1;
      result.encoding = "mono8";
      break;
    default:
      throw std::invalid_argument("Unsupported camera pixel encoding");
  }
  const uint64_t payload_size = static_cast<uint64_t>(message.step()) * message.height();
  if (message.step() < message.width() * channels ||
    payload_size > 4U * 1024U * 1024U || payload_size != message.data().size())
  {
    throw std::invalid_argument("Camera image stride/data length is invalid");
  }
  result.is_bigendian = 0;  // All accepted encodings contain single-byte channels.
  result.step = message.step();
  result.data.assign(message.data().begin(), message.data().end());
  return result;
}

sensor_msgs::msg::CameraInfo explicit_sdf_camera_info(const std_msgs::msg::Header & header)
{
  sensor_msgs::msg::CameraInfo result;
  result.header = header;
  result.width = kWidth;
  result.height = kHeight;
  result.distortion_model = "plumb_bob";
  result.d.assign(5, 0.0);
  result.k = {kFocalLength, 0.0, kPrincipalX, 0.0, kFocalLength, kPrincipalY, 0.0, 0.0, 1.0};
  result.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  result.p = {kFocalLength, 0.0, kPrincipalX, 0.0, 0.0, kFocalLength, kPrincipalY, 0.0,
    0.0, 0.0, 1.0, 0.0};
  return result;
}

sensor_msgs::msg::CameraInfo to_ros_camera_info(
  const gz::msgs::CameraInfo & message, const std::string & optical_frame)
{
  validate_shape(message.width(), message.height(), optical_frame);
  if (!message.has_header() || message.intrinsics().k_size() != 9 ||
    message.projection().p_size() != 12 ||
    (message.rectification_matrix_size() != 0 && message.rectification_matrix_size() != 9))
  {
    throw std::invalid_argument("Gazebo camera calibration header/matrices are incomplete");
  }
  std_msgs::msg::Header header;
  header.stamp = acquisition_stamp(message.header());
  header.frame_id = optical_frame;
  auto result = explicit_sdf_camera_info(header);
  std::copy(message.intrinsics().k().begin(), message.intrinsics().k().end(), result.k.begin());
  std::copy(message.projection().p().begin(), message.projection().p().end(), result.p.begin());
  if (message.rectification_matrix_size() == 9) {
    std::copy(message.rectification_matrix().begin(), message.rectification_matrix().end(), result.r.begin());
  }
  if (!message.has_distortion() ||
    message.distortion().model() != gz::msgs::CameraInfo::Distortion::PLUMB_BOB ||
    message.distortion().k_size() != 5)
  {
    throw std::invalid_argument("R2 requires the SDF zero-distortion pinhole camera model");
  }
  result.d.assign(message.distortion().k().begin(), message.distortion().k().end());
  for (const auto value : result.k) {
    if (!std::isfinite(value)) {throw std::invalid_argument("Nonfinite camera intrinsics");}
  }
  for (const auto value : result.p) {
    if (!std::isfinite(value)) {throw std::invalid_argument("Nonfinite camera projection");}
  }
  for (const auto value : result.r) {
    if (!std::isfinite(value)) {throw std::invalid_argument("Nonfinite camera rectification");}
  }
  for (const auto value : result.d) {
    if (!std::isfinite(value)) {throw std::invalid_argument("Nonfinite camera distortion");}
  }
  return result;
}

bool matches_explicit_sdf_calibration(const sensor_msgs::msg::CameraInfo & info)
{
  const auto expected = explicit_sdf_camera_info(info.header);
  const auto near = [](double actual, double wanted) {
      return std::isfinite(actual) && std::abs(actual - wanted) <= 1.0e-6;
    };
  return info.width == kWidth && info.height == kHeight &&
         info.distortion_model == "plumb_bob" && info.d.size() == expected.d.size() &&
         std::equal(info.k.begin(), info.k.end(), expected.k.begin(), near) &&
         std::equal(info.p.begin(), info.p.end(), expected.p.begin(), near) &&
         std::equal(info.r.begin(), info.r.end(), expected.r.begin(), near) &&
         std::equal(info.d.begin(), info.d.end(), expected.d.begin(), near);
}
}  // namespace r2
}  // namespace xq_gz_bridge
