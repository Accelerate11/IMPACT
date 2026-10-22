#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "xq_gz_bridge/camera_conversions.hpp"

namespace xq_gz_bridge
{
class R2CameraBridgeNode : public rclcpp::Node
{
public:
  R2CameraBridgeNode() : Node("impact_r2_camera_bridge")
  {
    const auto sensor_qos = rclcpp::QoS(20).reliable();
    const auto info_qos = rclcpp::QoS(20).reliable();
    for (size_t index = 0; index < streams_.size(); ++index) {
      auto & stream = streams_[index];
      stream.name = index == 0 ? "front" : "down";
      stream.frame = "impact_" + stream.name + "_optical";
      stream.gz_image_topic = "/xq/r2/" + stream.name + "/image";
      stream.gz_info_topic = "/xq/r2/" + stream.name + "/camera_info";
      stream.image_pub = create_publisher<sensor_msgs::msg::Image>(
        "/impact/r2/" + stream.name + "/image_raw", sensor_qos);
      stream.info_pub = create_publisher<sensor_msgs::msg::CameraInfo>(
        "/impact/r2/" + stream.name + "/camera_info", info_qos);
    }
    metadata_pub_ = create_publisher<std_msgs::msg::String>(
      "/impact/r2/calibration_metadata", rclcpp::QoS(1).reliable().transient_local());
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/impact/r2/camera_bridge/status", rclcpp::QoS(1).reliable().transient_local());
    // Subscribe to native CameraInfo as well as pixels, so Gazebo publishes
    // its actual rendered calibration. The fixed SDF calibration is only an
    // explicitly identified startup fallback before the first native record.
    if (!gz_node_.Subscribe(streams_[0].gz_info_topic, &R2CameraBridgeNode::front_info, this) ||
      !gz_node_.Subscribe(streams_[1].gz_info_topic, &R2CameraBridgeNode::down_info, this) ||
      !gz_node_.Subscribe(streams_[0].gz_image_topic, &R2CameraBridgeNode::front_image, this) ||
      !gz_node_.Subscribe(streams_[1].gz_image_topic, &R2CameraBridgeNode::down_image, this))
    {
      throw std::runtime_error("Cannot subscribe to the isolated R2 Gazebo cameras");
    }
    std_msgs::msg::String metadata;
    metadata.data = R"json({"schema_version":1,"simulation_calibration":true,"real_camera_calibration":false,"calibration_source":"gazebo_camera_info_with_explicit_sdf_startup_fallback","world":"xq_r2_multisensor","model":"xq_iris_multisensor_ardupilot","width":320,"height":240,"rate_hz":15,"horizontal_fov_rad":1.0471975512,"fx":277.128129211,"fy":277.128129211,"cx":159.5,"cy":119.5,"pixel_payload_modified":false,"timestamp_policy":"gazebo_acquisition_time_no_receive_time_substitution","ground_truth_used":false,"front":{"frame_id":"impact_front_optical","translation_body_m":[0.12,0,0.03],"R_body_optical":[[0,0,1],[-1,0,0],[0,-1,0]]},"down":{"frame_id":"impact_down_optical","translation_body_m":[0,0,-0.03],"R_body_optical":[[0,-1,0],[-1,0,0],[0,0,-1]]}})json";
    const auto context = declare_parameter<std::string>("calibration_context", "r2_base");
    if (context == "r2b3_range") {
      // Intrinsics are unchanged, while provenance names the actual new payload/world.
      const std::string old_world = "\"world\":\"xq_r2_multisensor\"";
      const std::string old_model = "\"model\":\"xq_iris_multisensor_ardupilot\"";
      metadata.data.replace(metadata.data.find(old_world), old_world.size(),
        "\"world\":\"xq_r2b3_range\"");
      metadata.data.replace(metadata.data.find(old_model), old_model.size(),
        "\"model\":\"xq_iris_multisensor_range_ardupilot\"");
    } else if (context != "r2_base") {
      throw std::invalid_argument("Unknown simulation calibration context");
    }
    metadata_pub_->publish(metadata);
    status_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {publish_status();});
    RCLCPP_INFO(get_logger(), "R2 camera bridge ready: original Gazebo pixels, acquisition stamps, simulation calibration");
  }

  ~R2CameraBridgeNode() override
  {
    stopping_.store(true);
    for (const auto & stream : streams_) {
      (void)gz_node_.Unsubscribe(stream.gz_image_topic);
      (void)gz_node_.Unsubscribe(stream.gz_info_topic);
    }
    // Drain a callback that entered immediately before the stopping flag.
    std::lock_guard<std::mutex> lock(mutex_);
  }

private:
  struct Stream
  {
    std::string name, frame, gz_image_topic, gz_info_topic;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub;
    bool native_info_seen = false;
    bool calibration_blocked = false;
    uint64_t images_received = 0, images_published = 0, images_rejected = 0;
    uint64_t native_infos_received = 0, native_infos_rejected = 0;
    uint64_t fallback_infos_published = 0;
  };

  bool active() const
  {
    return !stopping_.load() && rclcpp::ok();
  }

  void front_info(const gz::msgs::CameraInfo & message) {on_info(0, message);}
  void down_info(const gz::msgs::CameraInfo & message) {on_info(1, message);}
  void front_image(const gz::msgs::Image & message) {on_image(0, message);}
  void down_image(const gz::msgs::Image & message) {on_image(1, message);}

  void on_info(size_t index, const gz::msgs::CameraInfo & message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active()) {return;}
    auto & stream = streams_[index];
    ++stream.native_infos_received;
    try {
      const auto info = r2::to_ros_camera_info(message, stream.frame);
      if (!r2::matches_explicit_sdf_calibration(info)) {
        throw std::invalid_argument("Native Gazebo K/P/D differs from the frozen R2 SDF calibration");
      }
      stream.native_info_seen = true;
      stream.calibration_blocked = false;
      stream.info_pub->publish(info);  // Keep the native acquisition stamp.
    } catch (const std::exception & error) {
      ++stream.native_infos_rejected;
      stream.calibration_blocked = true;
      if (active()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "%s camera calibration rejected: %s", stream.name.c_str(), error.what());
      }
    }
  }

  void on_image(size_t index, const gz::msgs::Image & message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active()) {return;}
    auto & stream = streams_[index];
    ++stream.images_received;
    try {
      if (stream.calibration_blocked) {
        throw std::invalid_argument("Camera output blocked by a rejected native calibration");
      }
      const auto image = r2::to_ros_camera_image(message, stream.frame);
      if (!stream.native_info_seen) {
        stream.info_pub->publish(r2::explicit_sdf_camera_info(image.header));
        ++stream.fallback_infos_published;
      }
      stream.image_pub->publish(image);
      ++stream.images_published;
    } catch (const std::exception & error) {
      ++stream.images_rejected;
      if (active()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "%s camera image rejected: %s", stream.name.c_str(), error.what());
      }
    }
  }

  void publish_status()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active()) {return;}
    std::ostringstream json;
    json << "{\"schema_version\":1,\"ground_truth_used\":false,\"cameras\":{";
    for (size_t index = 0; index < streams_.size(); ++index) {
      const auto & stream = streams_[index];
      if (index != 0) {json << ',';}
      json << '"' << stream.name << "\":{\"images_received\":" << stream.images_received
           << ",\"images_published\":" << stream.images_published
           << ",\"images_rejected\":" << stream.images_rejected
           << ",\"native_infos_received\":" << stream.native_infos_received
           << ",\"native_infos_rejected\":" << stream.native_infos_rejected
           << ",\"fallback_infos_published\":" << stream.fallback_infos_published
           << ",\"native_info_seen\":" << (stream.native_info_seen ? "true" : "false")
           << ",\"calibration_blocked\":" << (stream.calibration_blocked ? "true" : "false") << '}';
    }
    json << "}}";
    std_msgs::msg::String status;
    status.data = json.str();
    try {
      status_pub_->publish(status);
    } catch (const rclcpp::exceptions::RCLError &) {
      if (active()) {throw;}
    }
  }

  std::atomic<bool> stopping_{false};
  std::mutex mutex_;
  std::array<Stream, 2> streams_;
  gz::transport::Node gz_node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metadata_pub_, status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};
}  // namespace xq_gz_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<xq_gz_bridge::R2CameraBridgeNode>();
  rclcpp::spin(node);
  node.reset();
  if (rclcpp::ok()) {rclcpp::shutdown();}
  return 0;
}
