#ifndef XQ_GZ_BRIDGE__RANGE_CONVERSIONS_HPP_
#define XQ_GZ_BRIDGE__RANGE_CONVERSIONS_HPP_

#include <cstdint>
#include <string>

#include <gz/msgs/imu.pb.h>
#include <gz/msgs/laserscan.pb.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/range.hpp>

namespace xq_gz_bridge
{
namespace r2b3
{
constexpr double kRangeMinimum = 0.05;
constexpr double kRangeMaximum = 10.0;
constexpr double kRangeVariance = 0.0001;
constexpr double kGyroVariance = 4.0e-8;
constexpr double kAccelerationVariance = 4.0e-6;
constexpr double kRangePeriod = 0.02;

// Strict acquisition time: never normalize malformed fields or substitute arrival time.
int64_t strict_acquisition_stamp_ns(const gz::msgs::Header & header);

// frame accepts an exact sensor name or a scope ending in ::name or /name.
// The SDF supplies the beam-axis contract; world_pose is intentionally not exported.
sensor_msgs::msg::LaserScan to_ros_pencil_scan(
  const gz::msgs::LaserScan & message, const std::string & expected_sensor_name);
sensor_msgs::msg::Range pencil_scan_to_range(const sensor_msgs::msg::LaserScan & scan);
sensor_msgs::msg::Imu to_ros_known_noise_imu(const gz::msgs::IMU & message);

// A single ray has no inter-ray angular spacing. Gazebo emits 0/0 NaN for this
// derived field; retain that fact separately while ROS uses increment=0.
bool has_undefined_single_sample_spacing(const gz::msgs::LaserScan & message);
std::string native_scan_source_json(
  const gz::msgs::LaserScan & message, const std::string & stream);

// Per-source stream. Validate a whole record before committing its acquisition stamp.
class StrictStampSequence
{
public:
  bool would_accept(int64_t stamp_ns) const;
  void commit(int64_t stamp_ns);
  bool has_stamp() const {return has_stamp_;}
  int64_t last_stamp_ns() const {return last_stamp_ns_;}

private:
  bool has_stamp_ = false;
  int64_t last_stamp_ns_ = 0;
};
}  // namespace r2b3
}  // namespace xq_gz_bridge

#endif  // XQ_GZ_BRIDGE__RANGE_CONVERSIONS_HPP_
