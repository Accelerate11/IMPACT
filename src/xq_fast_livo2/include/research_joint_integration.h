// Opt-in integration of raw downward images/range into the native LIVO state.
#ifndef IMPACT_RESEARCH_JOINT_INTEGRATION_H
#define IMPACT_RESEARCH_JOINT_INTEGRATION_H
#include <Eigen/Core>
#include <cstdint>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/core.hpp>
struct StatesGroup;
namespace impact_research {
struct EpochRange {
  bool available = false;
  double metres = 0., minimum = 0., maximum = 0.;
  std::string frame;
};
class JointIntegration {
 public:
  using Cov = Eigen::Matrix<double,19,19>;
  JointIntegration(const rclcpp::Node::SharedPtr& node, const std::string& directory);
  ~JointIntegration();
  void propagated(StatesGroup& state, const Cov& f, const Cov& q, const Cov& qgyro, bool imu_ready, int64_t stamp_ns);
  void native_update(const StatesGroup& before, StatesGroup& after, const std::string& phase, int64_t stamp_ns);
  void epoch(int64_t stamp_ns, StatesGroup& state, const cv::Mat& downward, const EpochRange& range);
  void input(const std::string& sensor, int64_t stamp_ns, bool accepted, const std::string& reason);
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace impact_research
#endif
