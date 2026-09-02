// Copyright 2026 Xuanqiong X1 Simulation Team
// SPDX-License-Identifier: Apache-2.0

#include <chrono>
#include <memory>

#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <sdf/Element.hh>

namespace xq_gz_bridge
{

class P12ObstacleMotionSystem final
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager &,
    gz::sim::EventManager &) override
  {
    model_ = gz::sim::Model(entity);
    x_m_ = sdf->Get<double>("x_m", x_m_).first;
    park_y_m_ = sdf->Get<double>("park_y_m", park_y_m_).first;
    blocked_y_m_ = sdf->Get<double>("blocked_y_m", blocked_y_m_).first;
    z_m_ = sdf->Get<double>("z_m", z_m_).first;
    enter_start_s_ = sdf->Get<double>("enter_start_s", enter_start_s_).first;
    enter_end_s_ = sdf->Get<double>("enter_end_s", enter_end_s_).first;
    leave_start_s_ = sdf->Get<double>("leave_start_s", leave_start_s_).first;
    leave_end_s_ = sdf->Get<double>("leave_end_s", leave_end_s_).first;
  }

  void PreUpdate(
    const gz::sim::UpdateInfo & info,
    gz::sim::EntityComponentManager & ecm) override
  {
    if (info.paused || !model_.Valid(ecm)) {
      return;
    }
    const double now_s = std::chrono::duration<double>(info.simTime).count();
    const double y_m = TrajectoryY(now_s);
    model_.SetWorldPoseCmd(ecm, gz::math::Pose3d(x_m_, y_m, z_m_, 0.0, 0.0, 0.0));
  }

private:
  double TrajectoryY(const double now_s) const
  {
    if (now_s < enter_start_s_) {
      return park_y_m_;
    }
    if (now_s < enter_end_s_) {
      const double ratio = (now_s - enter_start_s_) / (enter_end_s_ - enter_start_s_);
      return park_y_m_ + ratio * (blocked_y_m_ - park_y_m_);
    }
    if (now_s < leave_start_s_) {
      return blocked_y_m_;
    }
    if (now_s < leave_end_s_) {
      const double ratio = (now_s - leave_start_s_) / (leave_end_s_ - leave_start_s_);
      return blocked_y_m_ + ratio * (-park_y_m_ - blocked_y_m_);
    }
    return -park_y_m_;
  }

  gz::sim::Model model_{gz::sim::kNullEntity};
  double x_m_{-4.5};
  double park_y_m_{3.4};
  double blocked_y_m_{0.0};
  double z_m_{1.0};
  double enter_start_s_{18.0};
  double enter_end_s_{22.0};
  double leave_start_s_{36.0};
  double leave_end_s_{40.0};
};

}  // namespace xq_gz_bridge

GZ_ADD_PLUGIN(
  xq_gz_bridge::P12ObstacleMotionSystem,
  gz::sim::System,
  xq_gz_bridge::P12ObstacleMotionSystem::ISystemConfigure,
  xq_gz_bridge::P12ObstacleMotionSystem::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  xq_gz_bridge::P12ObstacleMotionSystem,
  "xq_gz_bridge::P12ObstacleMotionSystem")
