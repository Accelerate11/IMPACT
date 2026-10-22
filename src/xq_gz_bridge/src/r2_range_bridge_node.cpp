#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>

#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "xq_gz_bridge/range_conversions.hpp"

namespace xq_gz_bridge
{
namespace
{
// Native frame strings are data, not executable JSON fragments.
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

class R2RangeBridgeNode : public rclcpp::Node
{
public:
  R2RangeBridgeNode() : Node("impact_r2_range_bridge")
  {
    const auto sensor_qos = rclcpp::QoS(100).reliable();
    for (size_t index = 0; index < ranges_.size(); ++index) {
      auto & stream = ranges_[index];
      const bool noisy = index == 0;
      stream.name = noisy ? "noisy" : "reference";
      stream.sensor_name = "down_range_" + stream.name;
      stream.gz_topic = "/xq/r2b3/down_range/" + stream.name;
      stream.range_pub = create_publisher<sensor_msgs::msg::Range>(
        noisy ? "/impact/r2b3/down/range" : "/impact/r2b3/eval/range_reference", sensor_qos);
      stream.scan_pub = create_publisher<sensor_msgs::msg::LaserScan>(
        noisy ? "/impact/r2b3/native/noisy_scan" : "/impact/r2b3/eval/reference_scan", sensor_qos);
    }
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/impact/r2b3/imu", sensor_qos);
    source_ledger_pub_ = create_publisher<std_msgs::msg::String>(
      "/impact/r2b3/native/range_source", sensor_qos);
    metadata_pub_ = create_publisher<std_msgs::msg::String>(
      "/impact/r2b3/noise_metadata", rclcpp::QoS(1).reliable().transient_local());
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/impact/r2b3/range_bridge/status", rclcpp::QoS(1).reliable().transient_local());
    // Each subscription has an independent source-time watermark. A rewind needs a
    // new bridge process/recording; old and new epochs are never silently mixed.
    if (!gz_node_.Subscribe(ranges_[0].gz_topic, &R2RangeBridgeNode::on_noisy, this) ||
      !gz_node_.Subscribe(ranges_[1].gz_topic, &R2RangeBridgeNode::on_reference, this) ||
      !gz_node_.Subscribe("/xq/p4/imu", &R2RangeBridgeNode::on_imu, this))
    {
      throw std::runtime_error("Cannot subscribe to isolated R2B3B1 native range/IMU streams");
    }
    std_msgs::msg::String metadata;
    metadata.data = R"json({"schema_version":1,"stage":"R2B3B1","range_source_topic":"/impact/r2b3/native/range_source","single_sample_angular_step_policy":{"native_zero":"already_zero","native_nan":"single_sample_spacing_undefined_only_when_counts_equal_one_and_all_angle_endpoints_finite_zero","ros_angle_increment":0.0,"inf_or_nonzero":"reject","range_measurement_modified":false,"source_ledger_topic":"/impact/r2b3/native/range_source","undefined_step_status_counts":"validated_native_messages_including_any_later_publish_error"},"world":"xq_r2b3_range","model_definition":"xq_iris_multisensor_range_ardupilot","model_instance":"xq_p4_uav1","simulation_only":true,"acquisition_clock_domain":"gazebo_simulation_acquisition","timestamp_policy":"native_gazebo_header_strictly_increasing_per_stream_no_receive_time_substitution","reference_evaluation_only":true,"ground_truth_used_in_algorithm":false,"feature_depth_from_single_beam":false,"metric_flow_fusion_ready":false,"hardware_calibration_validated":false,"range":{"single_sample_angular_step_policy":"finite_zero_or_nan_only_for_one_by_one_zero_span_pencil; derived_ros_increment_zero; retain_original_fields_per_sample","native_type":"gz.msgs.LaserScan","ros_frame_id":"impact_down_range","native_sensor_names":["down_range_noisy","down_range_reference"],"native_frame_policy":"exact_sensor_name_or_scoped_sensor_suffix","translation_body_m":[0,0,-0.03],"rpy_body_range_rad":[0,1.5707963267948966,0],"R_body_range":[[0,0,1],[0,1,0],[-1,0,0]],"beam_direction_body":[0,0,-1],"extrinsics_source":"fixed_SDF_not_hardware_calibration","sample_count_horizontal":1,"sample_count_vertical":1,"angle_rad":0.0,"vertical_angle_rad":0.0,"field_of_view_rad":0.0,"field_of_view_model":"ideal_simulation_pencil","min_range_m":0.05,"max_range_m":10.0,"update_rate_hz":50.0,"scan_time_s":0.02,"time_increment_s":0.0,"range_noise_model":"gazebo_gaussian_per_sample_white","range_noise_mean_m":0.0,"range_noise_stddev_m":0.01,"range_noise_variance_m2":0.0001,"reference_noise":"none","variance_in_ros_range_message":false,"invalid_return_policy":"reject_nonfinite_range_or_angle_endpoints_out_of_bounds_or_wrong_contract_no_substitution","world_pose_discarded":true,"ros_float_conversion":"native_double_to_float32_same_for_scan_and_range","feature_depth_cross_covariance":"unknown","physical_bias_covariance":"unknown","reference_consumption_policy":"evaluation_only_never_estimator_or_flight_control"},"imu":{"native_topic":"/xq/p4/imu","ros_topic":"/impact/r2b3/imu","ros_frame_id":"livox_imu","original_topic_modified":false,"translation_body_m":[0,0,0],"R_body_imu":[[1,0,0],[0,1,0],[0,0,1]],"update_rate_hz":200.0,"orientation_covariance_known":false,"orientation_covariance_0":-1.0,"native_orientation_values_preserved_if_present":true,"angular_velocity_variance_per_axis":4e-8,"linear_acceleration_variance_per_axis":4e-6,"gyro_white_noise_stddev_rad_s":0.0002,"accel_white_noise_stddev_m_s2":0.002,"noise_parameter_semantics":"per_sample_measurement_variance_not_noise_density","covariance_source":"existing_SDF_independent_axis_gaussian_white_noise","physical_bias_covariance":"unknown","bias_feature_depth_cross_covariance":"unknown","real_hardware_calibration":false},"algorithm_state_updates":0,"flight_control_output":false})json";
    metadata_pub_->publish(metadata);
    publish_status();
    status_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {publish_status();});
    RCLCPP_INFO(get_logger(), "R2B3B1 range/known-white-noise IMU bridge ready; reference outputs are evaluation-only");
  }

  ~R2RangeBridgeNode() override
  {
    stopping_.store(true);
    (void)gz_node_.Unsubscribe(ranges_[0].gz_topic);
    (void)gz_node_.Unsubscribe(ranges_[1].gz_topic);
    (void)gz_node_.Unsubscribe("/xq/p4/imu");
    // Drain any transport callback that entered immediately before stopping.
    std::lock_guard<std::mutex> lock(mutex_);
  }

private:
  struct Counters
  {
    uint64_t received = 0, published = 0, rejected = 0;
    uint64_t timestamp_rejected = 0, validation_rejected = 0, publish_errors = 0;
    uint64_t undefined_angular_step_count = 0;
    uint64_t undefined_horizontal_step_count = 0, undefined_vertical_step_count = 0;
    std::string last_rejection, last_native_frame;
    r2b3::StrictStampSequence stamps;
  };

  struct RangeStream : Counters
  {
    std::string name, sensor_name, gz_topic;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
  };

  bool active() const {return !stopping_.load() && rclcpp::ok();}
  void on_noisy(const gz::msgs::LaserScan & message) {on_scan(0, message);}
  void on_reference(const gz::msgs::LaserScan & message) {on_scan(1, message);}

  bool source_time(Counters & counters, bool has_header, const gz::msgs::Header & header,
    int64_t & stamp_ns)
  {
    try {
      if (!has_header) {throw std::invalid_argument("Missing acquisition header");}
      stamp_ns = r2b3::strict_acquisition_stamp_ns(header);
      if (!counters.stamps.would_accept(stamp_ns)) {
        throw std::invalid_argument("Duplicate or regressing source acquisition stamp");
      }
      return true;
    } catch (const std::invalid_argument & error) {
      ++counters.rejected;
      ++counters.timestamp_rejected;
      counters.last_rejection = error.what();
      return false;
    }
  }

  void on_scan(size_t index, const gz::msgs::LaserScan & message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active()) {return;}
    auto & stream = ranges_[index];
    ++stream.received;
    stream.last_native_frame = message.frame();
    int64_t stamp_ns = 0;
    if (!source_time(stream, message.has_header(), message.header(), stamp_ns)) {return;}
    try {
      const auto scan = r2b3::to_ros_pencil_scan(message, stream.sensor_name);
      const auto range = r2b3::pencil_scan_to_range(scan);
      std_msgs::msg::String source_record;
      source_record.data = r2b3::native_scan_source_json(message, stream.name);
      stream.stamps.commit(stamp_ns);
      // Count validated native messages, even if a subsequent ROS publish fails.
      if (r2b3::has_undefined_single_sample_spacing(message)) {
        ++stream.undefined_angular_step_count;
      }
      if (std::isnan(message.angle_step())) {++stream.undefined_horizontal_step_count;}
      if (std::isnan(message.vertical_angle_step())) {++stream.undefined_vertical_step_count;}
      // Native scan and Range carry the exact same float32 range and source time.
      source_ledger_pub_->publish(source_record);
      stream.scan_pub->publish(scan);
      stream.range_pub->publish(range);
      ++stream.published;
    } catch (const rclcpp::exceptions::RCLError & error) {
      ++stream.rejected;
      ++stream.publish_errors;
      stream.last_rejection = error.what();
    } catch (const std::invalid_argument & error) {
      ++stream.rejected;
      ++stream.validation_rejected;
      stream.last_rejection = error.what();
      if (active()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "%s range rejected: %s", stream.name.c_str(), error.what());
      }
    }
  }

  void on_imu(const gz::msgs::IMU & message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active()) {return;}
    ++imu_.received;
    imu_.last_native_frame = message.entity_name();
    int64_t stamp_ns = 0;
    if (!source_time(imu_, message.has_header(), message.header(), stamp_ns)) {return;}
    try {
      const auto converted = r2b3::to_ros_known_noise_imu(message);
      imu_.stamps.commit(stamp_ns);
      imu_pub_->publish(converted);
      ++imu_.published;
    } catch (const rclcpp::exceptions::RCLError & error) {
      ++imu_.rejected;
      ++imu_.publish_errors;
      imu_.last_rejection = error.what();
    } catch (const std::invalid_argument & error) {
      ++imu_.rejected;
      ++imu_.validation_rejected;
      imu_.last_rejection = error.what();
      if (active()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "R2B3B1 IMU rejected: %s", error.what());
      }
    }
  }

  void append_status(std::ostringstream & json, const Counters & counters)
  {
    json << "{\"received\":" << counters.received
         << ",\"published\":" << counters.published
         << ",\"rejected\":" << counters.rejected
         << ",\"timestamp_rejected\":" << counters.timestamp_rejected
         << ",\"validation_rejected\":" << counters.validation_rejected
         << ",\"publish_errors\":" << counters.publish_errors
         << ",\"undefined_angular_step_count\":" << counters.undefined_angular_step_count
         << ",\"undefined_horizontal_step_count\":" << counters.undefined_horizontal_step_count
         << ",\"undefined_vertical_step_count\":" << counters.undefined_vertical_step_count
         << ",\"last_native_frame\":" << json_string(counters.last_native_frame)
         << ",\"last_rejection\":" << json_string(counters.last_rejection)
         << ",\"last_valid_source_stamp_ns\":";
    if (counters.stamps.has_stamp()) {json << counters.stamps.last_stamp_ns();}
    else {json << "null";}
    json << '}';
  }

  void publish_status()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active()) {return;}
    std::ostringstream json;
    json << "{\"schema_version\":1,\"acquisition_clock_domain\":\"gazebo_simulation_acquisition\","
      "\"reference_evaluation_only\":true,\"ground_truth_used_in_algorithm\":false,"
      "\"metric_flow_fusion_ready\":false,\"clock_rewind_policy\":\"reject_until_process_restart\",\"streams\":{\"noisy\":";
    append_status(json, ranges_[0]);
    json << ",\"reference\":";
    append_status(json, ranges_[1]);
    json << ",\"imu\":";
    append_status(json, imu_);
    json << "}}";
    std_msgs::msg::String status;
    status.data = json.str();
    try {status_pub_->publish(status);}
    catch (const rclcpp::exceptions::RCLError &) {if (active()) {throw;}}
  }

  std::atomic<bool> stopping_{false};
  std::mutex mutex_;
  std::array<RangeStream, 2> ranges_;
  Counters imu_;
  gz::transport::Node gz_node_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr source_ledger_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metadata_pub_, status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};
}  // namespace xq_gz_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<xq_gz_bridge::R2RangeBridgeNode>();
  rclcpp::spin(node);
  node.reset();
  if (rclcpp::ok()) {rclcpp::shutdown();}
  return 0;
}
