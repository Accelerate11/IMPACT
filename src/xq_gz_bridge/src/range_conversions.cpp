#include "xq_gz_bridge/range_conversions.hpp"

#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace xq_gz_bridge
{
namespace r2b3
{
namespace
{
bool frame_matches(const std::string & actual, const std::string & expected)
{
  if (expected.empty()) {return false;}
  if (actual == expected) {return true;}
  for (const auto & delimiter : {std::string("::"), std::string("/")}) {
    const auto suffix = delimiter + expected;
    if (actual.size() > suffix.size() &&
      actual.compare(actual.size() - suffix.size(), suffix.size(), suffix) == 0)
    {
      return true;
    }
  }
  return false;
}

bool equal_contract(double actual, double expected)
{
  return std::isfinite(actual) && std::abs(actual - expected) <= 1.0e-12;
}

builtin_interfaces::msg::Time strict_stamp(const gz::msgs::Header & header)
{
  (void)strict_acquisition_stamp_ns(header);
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(header.stamp().sec());
  stamp.nanosec = static_cast<uint32_t>(header.stamp().nsec());
  return stamp;
}

void require_finite(double value)
{
  if (!std::isfinite(value)) {throw std::invalid_argument("Nonfinite sensor value");}
}

std::string json_string(const std::string & value)
{
  static const char hex[] = "0123456789abcdef";
  std::string result = "\"";
  for (const unsigned char ch : value) {
    if (ch == '"' || ch == '\\') {result += '\\'; result += static_cast<char>(ch);}
    else if (ch < 0x20 || ch >= 0x7f) {
      result += "\\u00";
      result += hex[ch >> 4];
      result += hex[ch & 0x0f];
    } else {result += static_cast<char>(ch);}
  }
  result += '"';
  return result;
}
}  // namespace

int64_t strict_acquisition_stamp_ns(const gz::msgs::Header & header)
{
  if (!header.has_stamp() || header.stamp().sec() < 0 ||
    header.stamp().sec() > std::numeric_limits<int32_t>::max() ||
    header.stamp().nsec() < 0 || header.stamp().nsec() >= 1000000000)
  {
    throw std::invalid_argument("Missing or invalid Gazebo acquisition timestamp");
  }
  return header.stamp().sec() * 1000000000LL + header.stamp().nsec();
}

sensor_msgs::msg::LaserScan to_ros_pencil_scan(
  const gz::msgs::LaserScan & message, const std::string & expected_sensor_name)
{
  if (!message.has_header()) {
    throw std::invalid_argument("Range scan lacks acquisition header");
  }
  if (!frame_matches(message.frame(), expected_sensor_name) ||
    message.count() != 1 || message.vertical_count() != 1 ||
    message.ranges_size() != 1 ||
    (message.intensities_size() != 0 && message.intensities_size() != 1))
  {
    throw std::invalid_argument("Range native frame or one-by-one pencil shape differs from SDF");
  }
  for (const auto angle : {message.angle_min(), message.angle_max(),
      message.vertical_angle_min(), message.vertical_angle_max()})
  {
    if (!std::isfinite(angle) || angle != 0.0) {
      throw std::invalid_argument("Range beam angle is not the fixed zero-angle pencil");
    }
  }
  // Shape and finite zero endpoints have already been checked. With one sample,
  // inter-ray spacing is undefined (Gazebo computes 0/0); this exception never
  // applies to a measured range, timestamp, endpoint angle, or multi-ray scan.
  for (const auto step : {message.angle_step(), message.vertical_angle_step()}) {
    if (!std::isnan(step) && (!std::isfinite(step) || step != 0.0)) {
      throw std::invalid_argument("Single-pencil spacing must be zero or undefined NaN, never Inf/nonzero");
    }
  }
  if (!equal_contract(message.range_min(), kRangeMinimum) ||
    !equal_contract(message.range_max(), kRangeMaximum))
  {
    throw std::invalid_argument("Range bounds differ from fixed SDF");
  }
  const double range = message.ranges(0);
  require_finite(range);
  if (range < kRangeMinimum || range > kRangeMaximum) {
    throw std::invalid_argument("Range sample lies outside configured finite range");
  }
  if (message.intensities_size() == 1) {require_finite(message.intensities(0));}

  sensor_msgs::msg::LaserScan result;
  result.header.stamp = strict_stamp(message.header());
  result.header.frame_id = "impact_down_range";
  result.angle_min = result.angle_max = result.angle_increment = 0.0F;
  result.time_increment = 0.0F;
  result.scan_time = static_cast<float>(kRangePeriod);
  result.range_min = static_cast<float>(message.range_min());
  result.range_max = static_cast<float>(message.range_max());
  result.ranges.push_back(static_cast<float>(range));
  if (message.intensities_size() == 1) {
    const float intensity = static_cast<float>(message.intensities(0));
    if (!std::isfinite(intensity)) {
      throw std::invalid_argument("Native intensity overflows ROS float32");
    }
    result.intensities.push_back(intensity);
  }
  return result;
}

bool has_undefined_single_sample_spacing(const gz::msgs::LaserScan & message)
{
  return std::isnan(message.angle_step()) || std::isnan(message.vertical_angle_step());
}

std::string native_scan_source_json(
  const gz::msgs::LaserScan & message, const std::string & stream)
{
  if (stream != "noisy" && stream != "reference") {
    throw std::invalid_argument("Unknown native range stream");
  }
  // Ledger only validated observations; do not serialize a malformed measurement
  // or the transport world_pose. Validation also establishes the narrow NaN rule.
  (void)to_ros_pencil_scan(message, "down_range_" + stream);
  std::ostringstream json;
  json.imbue(std::locale::classic());
  json << std::setprecision(std::numeric_limits<double>::max_digits10);
  json << "{\"schema_version\":1,\"stream\":" << json_string(stream)
       << ",\"stamp_ns\":" << strict_acquisition_stamp_ns(message.header())
       << ",\"native_frame\":" << json_string(message.frame())
       << ",\"count\":" << message.count()
       << ",\"vertical_count\":" << message.vertical_count()
       << ",\"angle_min\":" << message.angle_min()
       << ",\"angle_max\":" << message.angle_max()
       << ",\"vertical_angle_min\":" << message.vertical_angle_min()
       << ",\"vertical_angle_max\":" << message.vertical_angle_max()
       << ",\"native_angle_step\":";
  if (std::isnan(message.angle_step())) {json << "\"nan\"";}
  else {json << message.angle_step();}
  json << ",\"native_vertical_angle_step\":";
  if (std::isnan(message.vertical_angle_step())) {json << "\"nan\"";}
  else {json << message.vertical_angle_step();}
  json << ",\"native_range_m\":" << message.ranges(0)
       << ",\"derived_ros_increment\":0,\"normalization_reason\":\""
       << (has_undefined_single_sample_spacing(message) ?
    "single_sample_spacing_undefined" : "already_zero") << "\"}";
  return json.str();
}

sensor_msgs::msg::Range pencil_scan_to_range(const sensor_msgs::msg::LaserScan & scan)
{
  if (scan.header.stamp.sec < 0 || scan.header.stamp.nanosec >= 1000000000U ||
    scan.header.frame_id != "impact_down_range" || scan.ranges.size() != 1 ||
    !std::isfinite(scan.ranges[0]) || scan.ranges[0] < scan.range_min ||
    scan.ranges[0] > scan.range_max || scan.angle_min != 0.0F ||
    scan.angle_max != 0.0F || scan.angle_increment != 0.0F ||
    scan.range_min != static_cast<float>(kRangeMinimum) ||
    scan.range_max != static_cast<float>(kRangeMaximum))
  {
    throw std::invalid_argument("Only a validated fixed-contract pencil scan can become Range");
  }
  sensor_msgs::msg::Range result;
  result.header = scan.header;
  result.radiation_type = sensor_msgs::msg::Range::INFRARED;
  result.field_of_view = 0.0F;  // Ideal simulation pencil, not physical sensor FOV.
  result.min_range = scan.range_min;
  result.max_range = scan.range_max;
  result.range = scan.ranges[0];
  return result;
}

sensor_msgs::msg::Imu to_ros_known_noise_imu(const gz::msgs::IMU & message)
{
  if (!message.has_header() || !message.has_angular_velocity() ||
    !message.has_linear_acceleration())
  {
    throw std::invalid_argument("IMU lacks acquisition header or measurement vectors");
  }
  const auto & gyro = message.angular_velocity();
  const auto & accel = message.linear_acceleration();
  for (const auto value : {gyro.x(), gyro.y(), gyro.z(), accel.x(), accel.y(), accel.z()}) {
    require_finite(value);
  }
  sensor_msgs::msg::Imu result;
  result.header.stamp = strict_stamp(message.header());
  result.header.frame_id = "livox_imu";
  if (message.has_orientation()) {
    const auto & orientation = message.orientation();
    for (const auto value : {orientation.x(), orientation.y(), orientation.z(), orientation.w()}) {
      require_finite(value);
    }
    result.orientation.x = orientation.x();
    result.orientation.y = orientation.y();
    result.orientation.z = orientation.z();
    result.orientation.w = orientation.w();
  } else {
    result.orientation.w = 1.0;
  }
  // Keep a present native quaternion as raw data, but never imply known orientation covariance.
  result.orientation_covariance[0] = -1.0;
  result.angular_velocity.x = gyro.x();
  result.angular_velocity.y = gyro.y();
  result.angular_velocity.z = gyro.z();
  result.linear_acceleration.x = accel.x();
  result.linear_acceleration.y = accel.y();
  result.linear_acceleration.z = accel.z();
  for (const auto diagonal : {0, 4, 8}) {
    result.angular_velocity_covariance[diagonal] = kGyroVariance;
    result.linear_acceleration_covariance[diagonal] = kAccelerationVariance;
  }
  return result;
}

bool StrictStampSequence::would_accept(int64_t stamp_ns) const
{
  return stamp_ns >= 0 && (!has_stamp_ || stamp_ns > last_stamp_ns_);
}

void StrictStampSequence::commit(int64_t stamp_ns)
{
  if (!would_accept(stamp_ns)) {
    throw std::invalid_argument("Acquisition timestamp is duplicate, regressing, or negative");
  }
  has_stamp_ = true;
  last_stamp_ns_ = stamp_ns;
}
}  // namespace r2b3
}  // namespace xq_gz_bridge
