#include <algorithm>
#include <atomic>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <gz/msgs/clock.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/transport/Node.hh>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "xq_gz_bridge/conversions.hpp"

namespace xq_gz_bridge
{

class XqGzBridgeNode : public rclcpp::Node
{
public:
  XqGzBridgeNode()
  : Node("xq_gz_bridge_node")
  {
    const auto gz_clock_topic =
      declare_parameter<std::string>("gz_clock_topic", "/clock");
    const auto ros_clock_topic =
      declare_parameter<std::string>("ros_clock_topic", "/clock");
    const auto gz_lidar_topic =
      declare_parameter<std::string>("gz_lidar_topic", "/xq/lidar/points");
    const auto ros_lidar_topic = declare_parameter<std::string>(
      "ros_lidar_topic", "/xq/agent_01/sensors/lidar/points");
    const auto gz_imu_topic =
      declare_parameter<std::string>("gz_imu_topic", "/xq/imu");
    const auto ros_imu_topic = declare_parameter<std::string>(
      "ros_imu_topic", "/xq/agent_01/sensors/imu");
    const auto gz_ground_truth_topic = declare_parameter<std::string>(
      "gz_ground_truth_topic", "/model/xq_agent_01/odometry");
    const auto ros_ground_truth_topic = declare_parameter<std::string>(
      "ros_ground_truth_topic", "/xq/eval/agent_01/ground_truth");
    const auto ros_cmd_vel_topic = declare_parameter<std::string>(
      "ros_cmd_vel_topic", "/xq/agent_01/cmd_vel");
    const auto gz_cmd_vel_topic = declare_parameter<std::string>(
      "gz_cmd_vel_topic", "/model/xq_agent_01/cmd_vel");

    lidar_frame_id_ =
      declare_parameter<std::string>("lidar_frame_id", "xq_mid360_link");
    imu_frame_id_ =
      declare_parameter<std::string>("imu_frame_id", "xq_imu_link");
    world_frame_id_ =
      declare_parameter<std::string>("world_frame_id", "xq_world");
    base_frame_id_ =
      declare_parameter<std::string>("base_frame_id", "xq_base_link");
    ground_truth_entity_name_ = declare_parameter<std::string>(
      "ground_truth_entity_name", "xq_agent_01");
    const auto ground_truth_source_type = declare_parameter<std::string>(
      "ground_truth_source_type", "odometry");
    const bool publish_ground_truth =
      declare_parameter<bool>("publish_ground_truth", true);

    random_seed_ = static_cast<uint64_t>(declare_parameter<int64_t>("random_seed", 20260822));
    lidar_dropout_ratio_ = declare_parameter<double>("lidar_dropout_ratio", 0.0);
    imu_dropout_ratio_ = declare_parameter<double>("imu_dropout_ratio", 0.0);
    lidar_jitter_ms_ = declare_parameter<double>("lidar_timestamp_jitter_ms", 0.0);
    imu_jitter_ms_ = declare_parameter<double>("imu_timestamp_jitter_ms", 0.0);
    motion_distortion_enabled_ =
      declare_parameter<bool>("motion_distortion_enabled", false);
    scan_period_s_ = declare_parameter<double>("lidar_scan_period_s", 0.1);
    const auto motion_velocity = declare_parameter<std::vector<double>>(
      "motion_linear_velocity_mps", {0.0, 0.0, 0.0});
    const auto extrinsic_xyz = declare_parameter<std::vector<double>>(
      "lidar_extrinsic_error_xyz_m", {0.0, 0.0, 0.0});
    const auto extrinsic_rpy = declare_parameter<std::vector<double>>(
      "lidar_extrinsic_error_rpy_rad", {0.0, 0.0, 0.0});
    motion_yaw_rate_rps_ = declare_parameter<double>("motion_yaw_rate_rps", 0.0);
    lidar_range_noise_std_m_ =
      declare_parameter<double>("lidar_range_noise_std_m", 0.015);
    validate_probability(lidar_dropout_ratio_, "lidar_dropout_ratio");
    validate_probability(imu_dropout_ratio_, "imu_dropout_ratio");
    if (lidar_jitter_ms_ < 0.0 || imu_jitter_ms_ < 0.0 || scan_period_s_ <= 0.0 ||
      lidar_range_noise_std_m_ < 0.0)
    {
      throw std::invalid_argument("P2 timing/noise parameters must be non-negative");
    }
    motion_velocity_mps_ = vector3(motion_velocity, "motion_linear_velocity_mps");
    extrinsic_xyz_m_ = vector3(extrinsic_xyz, "lidar_extrinsic_error_xyz_m");
    extrinsic_rpy_rad_ = vector3(extrinsic_rpy, "lidar_extrinsic_error_rpy_rad");

    clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>(
      ros_clock_topic, rclcpp::ClockQoS());
    // This is a local sensor-to-estimator link carrying ~0.7 MiB frames.  The
    // previous best-effort depth-5 writer delivered only about 6/10 Gazebo
    // frames even to the C++ rosbag2 recorder under WSL.  Reliability belongs
    // here (before any intentionally lossy inter-vehicle relay), and depth 20
    // absorbs short serialization / scheduling bursts without changing the
    // LiDAR samples or timestamps.
    lidar_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      ros_lidar_topic, rclcpp::QoS(rclcpp::KeepLast(20)).reliable());
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
      ros_imu_topic, rclcpp::SensorDataQoS());

    if (!gz_node_.Subscribe(gz_clock_topic, &XqGzBridgeNode::on_clock, this)) {
      throw std::runtime_error("Unable to subscribe to Gazebo clock topic: " + gz_clock_topic);
    }
    gz_subscription_topics_.push_back(gz_clock_topic);
    if (!gz_node_.Subscribe(gz_lidar_topic, &XqGzBridgeNode::on_point_cloud, this)) {
      throw std::runtime_error("Unable to subscribe to Gazebo LiDAR topic: " + gz_lidar_topic);
    }
    gz_subscription_topics_.push_back(gz_lidar_topic);
    if (!gz_node_.Subscribe(gz_imu_topic, &XqGzBridgeNode::on_imu, this)) {
      throw std::runtime_error("Unable to subscribe to Gazebo IMU topic: " + gz_imu_topic);
    }
    gz_subscription_topics_.push_back(gz_imu_topic);

    if (publish_ground_truth) {
      ground_truth_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        ros_ground_truth_topic, rclcpp::QoS(20).reliable());
      if (ground_truth_source_type == "odometry") {
        if (!gz_node_.Subscribe(
            gz_ground_truth_topic, &XqGzBridgeNode::on_ground_truth_odometry, this))
        {
          throw std::runtime_error(
                  "Unable to subscribe to Gazebo ground-truth odometry topic: " +
                  gz_ground_truth_topic);
        }
        gz_subscription_topics_.push_back(gz_ground_truth_topic);
      } else if (ground_truth_source_type == "pose_v") {
        if (!gz_node_.Subscribe(
            gz_ground_truth_topic, &XqGzBridgeNode::on_ground_truth_pose_v, this))
        {
          throw std::runtime_error(
                  "Unable to subscribe to Gazebo ground-truth Pose_V topic: " +
                  gz_ground_truth_topic);
        }
        gz_subscription_topics_.push_back(gz_ground_truth_topic);
      } else {
        throw std::invalid_argument(
                "ground_truth_source_type must be 'odometry' or 'pose_v', got: " +
                ground_truth_source_type);
      }
    }

    gz_cmd_vel_pub_ = gz_node_.Advertise<gz::msgs::Twist>(gz_cmd_vel_topic);
    if (!gz_cmd_vel_pub_) {
      throw std::runtime_error("Unable to advertise Gazebo cmd_vel topic: " + gz_cmd_vel_topic);
    }
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      ros_cmd_vel_topic,
      rclcpp::QoS(10).reliable(),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr message) {
        if (!callbacks_enabled()) {
          return;
        }
        auto publisher = gz_cmd_vel_pub_;
        const auto gz_message = to_gz_twist(*message);
        if (!publisher || !publisher.Publish(gz_message)) {
          if (!callbacks_enabled()) {
            return;
          }
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Gazebo rejected a cmd_vel publication");
        }
      });

    RCLCPP_INFO(
      get_logger(),
      "XQ bridge ready: lidar %s -> %s, IMU %s -> %s, cmd_vel %s -> %s",
      gz_lidar_topic.c_str(), ros_lidar_topic.c_str(),
      gz_imu_topic.c_str(), ros_imu_topic.c_str(),
      ros_cmd_vel_topic.c_str(), gz_cmd_vel_topic.c_str());
    RCLCPP_INFO(
      get_logger(),
      "P2 conditioning: range_noise_std=%.4f m, lidar_dropout=%.3f, imu_dropout=%.3f, "
      "lidar_jitter=%.3f ms, imu_jitter=%.3f ms, motion_distortion=%s",
      lidar_range_noise_std_m_, lidar_dropout_ratio_, imu_dropout_ratio_,
      lidar_jitter_ms_, imu_jitter_ms_, motion_distortion_enabled_ ? "true" : "false");
    if (publish_ground_truth) {
      RCLCPP_WARN(
        get_logger(),
        "Ground truth is enabled on evaluation-only topic %s; autonomy nodes must not subscribe",
        ros_ground_truth_topic.c_str());
    }
  }

  ~XqGzBridgeNode() override
  {
    stopping_.store(true, std::memory_order_release);

    // Break the ROS -> Gazebo path before tearing down its Gazebo publisher.
    cmd_vel_sub_.reset();

    // Stop Gazebo worker threads from entering callbacks that use ROS publishers.
    for (const auto & topic : gz_subscription_topics_) {
      (void)gz_node_.Unsubscribe(topic);
    }
  }

private:
  static void validate_probability(double value, const char * name)
  {
    if (value < 0.0 || value >= 1.0) {
      throw std::invalid_argument(std::string(name) + " must be in [0, 1)");
    }
  }

  static std::array<double, 3> vector3(
    const std::vector<double> & value, const char * name)
  {
    if (value.size() != 3U) {
      throw std::invalid_argument(std::string(name) + " must contain exactly 3 values");
    }
    return {value[0], value[1], value[2]};
  }

  static uint64_t mix64(uint64_t value)
  {
    value += 0x9e3779b97f4a7c15ULL;
    value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
    value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
    return value ^ (value >> 31U);
  }

  double deterministic_unit(uint64_t sequence, uint64_t stream_tag) const
  {
    const uint64_t bits = mix64(random_seed_ ^ mix64(sequence) ^ stream_tag);
    return static_cast<double>(bits >> 11U) * (1.0 / 9007199254740992.0);
  }

  bool drop_sample(double ratio, uint64_t sequence, uint64_t stream_tag) const
  {
    return ratio > 0.0 && deterministic_unit(sequence, stream_tag) < ratio;
  }

  builtin_interfaces::msg::Time conditioned_stamp(
    const builtin_interfaces::msg::Time & input,
    double jitter_ms,
    uint64_t sequence,
    uint64_t stream_tag,
    int64_t & last_stamp_ns,
    std::mutex & stamp_mutex)
  {
    constexpr int64_t ns_per_second = 1000000000LL;
    const int64_t base_ns = static_cast<int64_t>(input.sec) * ns_per_second + input.nanosec;
    const double signed_unit = 2.0 * deterministic_unit(sequence, stream_tag) - 1.0;
    const int64_t jitter_ns = static_cast<int64_t>(
      std::llround(signed_unit * jitter_ms * 1000000.0));
    std::lock_guard<std::mutex> lock(stamp_mutex);
    int64_t output_ns = base_ns + jitter_ns;
    if (last_stamp_ns != std::numeric_limits<int64_t>::min() && output_ns <= last_stamp_ns) {
      output_ns = last_stamp_ns + 1;
    }
    last_stamp_ns = output_ns;
    builtin_interfaces::msg::Time output;
    output.sec = static_cast<int32_t>(output_ns / ns_per_second);
    output.nanosec = static_cast<uint32_t>(output_ns % ns_per_second);
    return output;
  }

  void condition_point_cloud(sensor_msgs::msg::PointCloud2 & cloud)
  {
    const bool extrinsic_enabled =
      std::any_of(extrinsic_xyz_m_.begin(), extrinsic_xyz_m_.end(),
        [](double value) {return std::abs(value) > 0.0;}) ||
      std::any_of(extrinsic_rpy_rad_.begin(), extrinsic_rpy_rad_.end(),
        [](double value) {return std::abs(value) > 0.0;});
    if (!motion_distortion_enabled_ && !extrinsic_enabled) {
      return;
    }

    const double cr = std::cos(extrinsic_rpy_rad_[0]);
    const double sr = std::sin(extrinsic_rpy_rad_[0]);
    const double cp = std::cos(extrinsic_rpy_rad_[1]);
    const double sp = std::sin(extrinsic_rpy_rad_[1]);
    const double cy = std::cos(extrinsic_rpy_rad_[2]);
    const double sy = std::sin(extrinsic_rpy_rad_[2]);
    const std::array<std::array<double, 3>, 3> rotation{{
      {{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr}},
      {{sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr}},
      {{-sp, cp * sr, cp * cr}}
    }};

    try {
      sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
      const size_t count = static_cast<size_t>(cloud.width) * cloud.height;
      size_t index = 0U;
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++index) {
        double x = *iter_x;
        double y = *iter_y;
        double z = *iter_z;
        if (motion_distortion_enabled_ && count > 1U) {
          const double fraction = static_cast<double>(index) /
            static_cast<double>(count - 1U);
          const double tau = (fraction - 1.0) * scan_period_s_;
          const double yaw = motion_yaw_rate_rps_ * tau;
          const double cos_yaw = std::cos(yaw);
          const double sin_yaw = std::sin(yaw);
          const double moved_x = cos_yaw * x - sin_yaw * y + motion_velocity_mps_[0] * tau;
          const double moved_y = sin_yaw * x + cos_yaw * y + motion_velocity_mps_[1] * tau;
          x = moved_x;
          y = moved_y;
          z += motion_velocity_mps_[2] * tau;
        }
        const double transformed_x =
          rotation[0][0] * x + rotation[0][1] * y + rotation[0][2] * z + extrinsic_xyz_m_[0];
        const double transformed_y =
          rotation[1][0] * x + rotation[1][1] * y + rotation[1][2] * z + extrinsic_xyz_m_[1];
        const double transformed_z =
          rotation[2][0] * x + rotation[2][1] * y + rotation[2][2] * z + extrinsic_xyz_m_[2];
        *iter_x = static_cast<float>(transformed_x);
        *iter_y = static_cast<float>(transformed_y);
        *iter_z = static_cast<float>(transformed_z);
      }
    } catch (const std::runtime_error & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Cannot apply P2 point-cloud conditioning: %s", error.what());
    }
  }

  bool callbacks_enabled() const
  {
    return !stopping_.load(std::memory_order_acquire) && rclcpp::ok();
  }

  template<typename PublisherT, typename MessageT>
  void publish_to_ros_safely(
    const std::shared_ptr<PublisherT> & publisher,
    MessageT && message,
    const char * stream_name)
  {
    if (!callbacks_enabled() || !publisher) {
      return;
    }

    // Hold a local publisher reference until this callback has returned. The
    // rclcpp context can still become invalid after the check, so RCLError is
    // an expected shutdown race and must not escape a Gazebo worker thread.
    auto held_publisher = publisher;
    try {
      held_publisher->publish(std::forward<MessageT>(message));
    } catch (const rclcpp::exceptions::RCLError & error) {
      if (callbacks_enabled()) {
        RCLCPP_ERROR(
          get_logger(), "Failed to publish %s message: %s", stream_name, error.what());
      }
    }
  }

  template<typename MessageT>
  bool has_sample_stamp(const MessageT & message, const char * stream_name)
  {
    if (message.has_header() && message.header().has_stamp()) {
      return true;
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Dropping %s message without a Gazebo sample timestamp",
      stream_name);
    return false;
  }

  void on_clock(const gz::msgs::Clock & message)
  {
    if (!callbacks_enabled() || !message.has_sim()) {
      return;
    }
    rosgraph_msgs::msg::Clock output;
    output.clock = to_ros_time(message.sim());
    publish_to_ros_safely(clock_pub_, std::move(output), "clock");
  }

  void on_point_cloud(const gz::msgs::PointCloudPacked & message)
  {
    if (!callbacks_enabled() || !has_sample_stamp(message, "LiDAR")) {
      return;
    }
    const uint64_t sequence = lidar_sequence_.fetch_add(1U, std::memory_order_relaxed);
    if (drop_sample(lidar_dropout_ratio_, sequence, 0x4c49444152ULL)) {
      return;
    }
    auto output = to_ros_point_cloud(message, lidar_frame_id_, builtin_interfaces::msg::Time{});
    output.header.stamp = conditioned_stamp(
      output.header.stamp, lidar_jitter_ms_, sequence, 0x4c4a495454ULL,
      last_lidar_stamp_ns_, lidar_stamp_mutex_);
    condition_point_cloud(output);
    publish_to_ros_safely(lidar_pub_, std::move(output), "LiDAR");
  }

  void on_imu(const gz::msgs::IMU & message)
  {
    if (!callbacks_enabled() || !has_sample_stamp(message, "IMU")) {
      return;
    }
    const uint64_t sequence = imu_sequence_.fetch_add(1U, std::memory_order_relaxed);
    if (drop_sample(imu_dropout_ratio_, sequence, 0x494d5521ULL)) {
      return;
    }
    auto output = to_ros_imu(message, imu_frame_id_, builtin_interfaces::msg::Time{});
    output.header.stamp = conditioned_stamp(
      output.header.stamp, imu_jitter_ms_, sequence, 0x494a495454ULL,
      last_imu_stamp_ns_, imu_stamp_mutex_);
    publish_to_ros_safely(imu_pub_, std::move(output), "IMU");
  }

  void on_ground_truth_odometry(const gz::msgs::Odometry & message)
  {
    if (!callbacks_enabled() || !has_sample_stamp(message, "ground-truth odometry")) {
      return;
    }
    publish_to_ros_safely(
      ground_truth_pub_,
      to_ros_odometry(
        message, world_frame_id_, base_frame_id_, builtin_interfaces::msg::Time{}),
      "ground-truth odometry");
  }

  void on_ground_truth_pose_v(const gz::msgs::Pose_V & message)
  {
    if (!callbacks_enabled() || !has_sample_stamp(message, "ground-truth Pose_V")) {
      return;
    }
    const auto output = pose_v_to_ros_odometry(
      message,
      ground_truth_entity_name_,
      world_frame_id_,
      base_frame_id_,
      builtin_interfaces::msg::Time{});
    if (output) {
      publish_to_ros_safely(ground_truth_pub_, *output, "ground-truth Pose_V");
      return;
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Entity '%s' is absent from Gazebo Pose_V ground truth",
      ground_truth_entity_name_.c_str());
  }

  std::atomic_bool stopping_{false};
  std::atomic<uint64_t> lidar_sequence_{0U};
  std::atomic<uint64_t> imu_sequence_{0U};
  uint64_t random_seed_{20260822U};
  double lidar_dropout_ratio_{0.0};
  double imu_dropout_ratio_{0.0};
  double lidar_jitter_ms_{0.0};
  double imu_jitter_ms_{0.0};
  double lidar_range_noise_std_m_{0.015};
  bool motion_distortion_enabled_{false};
  double scan_period_s_{0.1};
  double motion_yaw_rate_rps_{0.0};
  std::array<double, 3> motion_velocity_mps_{{0.0, 0.0, 0.0}};
  std::array<double, 3> extrinsic_xyz_m_{{0.0, 0.0, 0.0}};
  std::array<double, 3> extrinsic_rpy_rad_{{0.0, 0.0, 0.0}};
  int64_t last_lidar_stamp_ns_{std::numeric_limits<int64_t>::min()};
  int64_t last_imu_stamp_ns_{std::numeric_limits<int64_t>::min()};
  std::mutex lidar_stamp_mutex_;
  std::mutex imu_stamp_mutex_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ground_truth_pub_;
  std::string lidar_frame_id_;
  std::string imu_frame_id_;
  std::string world_frame_id_;
  std::string base_frame_id_;
  std::string ground_truth_entity_name_;
  std::vector<std::string> gz_subscription_topics_;

  // Declaration order is intentional: reverse destruction stops ROS command
  // callbacks, then Gazebo publication/subscription, before ROS publishers.
  gz::transport::Node gz_node_;
  gz::transport::Node::Publisher gz_cmd_vel_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

}  // namespace xq_gz_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<xq_gz_bridge::XqGzBridgeNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("xq_gz_bridge_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
