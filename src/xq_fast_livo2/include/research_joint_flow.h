// Research-only stochastic pose cloning with a local, range-anchored plane.
// Eigen-only implementation.  It is an EKF model, not an integrity guarantee.
// Native LIO/VIO maps are treated as fixed and their observations as independent;
// historical map/state correlation is not reconstructed by this extension.
#ifndef IMPACT_RESEARCH_JOINT_FLOW_H
#define IMPACT_RESEARCH_JOINT_FLOW_H

#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace impact_research {
constexpr int kNative = 19, kClone = 19, kPlane = 25, kJoint = 28;
using NativeCov = Eigen::Matrix<double, kNative, kNative>;
using NativeDelta = Eigen::Matrix<double, kNative, 1>;
using JointCov = Eigen::Matrix<double, kJoint, kJoint>;
using JointDelta = Eigen::Matrix<double, kJoint, 1>;
using V3 = Eigen::Vector3d;
using M3 = Eigen::Matrix3d;

inline M3 skew(const V3& v) {
  M3 s; s << 0., -v.z(), v.y(), v.z(), 0., -v.x(), -v.y(), v.x(), 0.; return s;
}
inline M3 exp_so3(const V3& v) {
  const double a = v.norm();
  if (a < 1e-9) return M3::Identity() + skew(v) + .5 * skew(v) * skew(v);
  return Eigen::AngleAxisd(a, v / a).toRotationMatrix();
}
inline V3 log_so3(const M3& r) {
  const Eigen::AngleAxisd a(r); return a.axis() * a.angle();
}
// Right-multiplicative SO(3) error: R_true = R_nominal Exp(delta_theta).
inline M3 right_reset(const V3& d) {
  const double a = d.norm(); const M3 s = skew(d);
  if (a < 1e-6) return M3::Identity() - .5 * s + (1. / 6.) * s * s;
  return M3::Identity() - ((1. - std::cos(a)) / (a * a)) * s +
      ((a - std::sin(a)) / (a * a * a)) * s * s;
}
inline bool rotation_valid(const M3& r) {
  return r.allFinite() && (r.transpose() * r - M3::Identity()).norm() < 1e-6 &&
      std::abs(r.determinant() - 1.) < 1e-6;
}
template<class Derived> inline bool covariance_valid(const Eigen::MatrixBase<Derived>& p,
                                                    bool positive = false) {
  if (p.rows() != p.cols() || !p.allFinite()) return false;
  const double scale = std::max(1., p.norm());
  if ((p - p.transpose()).norm() > 1e-8 * scale) return false;
  const Eigen::MatrixXd symmetric = .5 * (p + p.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(symmetric, Eigen::EigenvaluesOnly);
  if (solver.info() != Eigen::Success) return false;
  return solver.eigenvalues().minCoeff() >= (positive ? 1e-14 : -1e-9 * scale);
}

struct Camera {
  double fx = 0., fy = 0., cx = 0., cy = 0.;
  M3 body_from_optical = M3::Identity();
  V3 optical_origin_body = V3::Zero();
  bool valid() const {
    return std::isfinite(fx) && std::isfinite(fy) && fx > 0. && fy > 0. &&
      std::isfinite(cx) && std::isfinite(cy) && rotation_valid(body_from_optical) &&
      optical_origin_body.allFinite();
  }
};
struct RangeRay {
  V3 origin_body = V3::Zero();
  V3 unit_direction_body = V3(0., 0., -1.);
  bool valid() const {
    return origin_body.allFinite() && unit_direction_body.allFinite() &&
      std::abs(unit_direction_body.norm() - 1.) < 1e-6;
  }
};
struct Track {
  Eigen::Vector2d first_pixel = Eigen::Vector2d::Zero();
  Eigen::Vector2d second_pixel = Eigen::Vector2d::Zero();
  // Covariances of the two raw endpoint pixels, after undistortion.  The caller
  // must calibrate/supply them; unknown covariance is never substituted.
  Eigen::Matrix2d first_cov = Eigen::Matrix2d::Zero();
  Eigen::Matrix2d second_cov = Eigen::Matrix2d::Zero();
  uint64_t id = 0;
};
struct Policy {
  double minimum_depth_m = .15, maximum_depth_m = 12.;
  double minimum_ray_plane_cosine = .25;
  double maximum_local_radius_m = 2.;
  double maximum_abs_slope = 1.;
  double minimum_translation_m = .005;
  double minimum_pixel_span = 15.;
  size_t minimum_tracks = 8, maximum_tracks = 80;
  double minimum_translation_information = 1e-8;
  double individual_nis_limit = 9.210340371976184; // chi-square(2), .99
  double range_nis_limit = 6.6348966010212145;     // chi-square(1), .99
  // Engineering batch cutoff, not a claimed chi-square quantile.
  double maximum_batch_nis_per_dof = 3.;
  double maximum_rotation_correction_rad = .2;
  double maximum_position_correction_m = .5;
  bool valid() const {
    const double values[] = {minimum_depth_m,maximum_depth_m,minimum_ray_plane_cosine,
      maximum_local_radius_m,maximum_abs_slope,minimum_translation_m,minimum_pixel_span,
      minimum_translation_information,individual_nis_limit,range_nis_limit,
      maximum_batch_nis_per_dof,maximum_rotation_correction_rad,maximum_position_correction_m};
    for (double value : values) if (!std::isfinite(value) || value <= 0.) return false;
    return maximum_depth_m > minimum_depth_m && minimum_ray_plane_cosine <= 1. &&
      minimum_tracks >= 3 && maximum_tracks >= minimum_tracks;
  }
};
struct Projection {
  bool valid = false;
  std::string reason;
  Eigen::Vector2d pixel = Eigen::Vector2d::Zero();
  Eigen::Matrix<double,2,kJoint> jacobian = Eigen::Matrix<double,2,kJoint>::Zero();
  Eigen::Matrix2d first_pixel_jacobian = Eigen::Matrix2d::Zero();
  V3 point_world = V3::Zero();
  double initial_optical_depth_m = 0., final_optical_depth_m = 0.;
};
struct Update {
  bool accepted = false;
  std::string reason;
  size_t input_tracks = 0, accepted_tracks = 0, rejected_tracks = 0;
  double nis = std::numeric_limits<double>::quiet_NaN();
  int dof = 0;
  double native_delta_norm = 0., covariance_delta_norm = 0.;
  JointDelta delta = JointDelta::Zero();
};
struct MeasurementDiagnostics {
  std::string kind;
  bool applied = false;
  Eigen::MatrixXd jacobian, noise, innovation, gain;
  Eigen::VectorXd residual;
  std::vector<uint64_t> accepted_track_ids;
  std::vector<Eigen::Vector2d> predicted_pixels;
};
enum class CrossAblation { None, ZeroGyroCross, ZeroPlaneCross };

// Native must expose StatesGroup-compatible fields: rot_end, pos_end, vel_end,
// inv_expo_time, bias_g, bias_a, gravity, cov.  No ROS type dependency is needed.
class JointFlow {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit JointFlow(Policy policy = Policy()) : policy_(policy) {
    if (!policy_.valid()) throw std::invalid_argument("invalid joint-flow policy");
  }
  bool initialized() const { return initialized_; }
  const JointCov& covariance() const { return p_; }
  const JointCov& gyro_covariance_component() const { return pg_; }
  const M3& clone_rotation() const { return r0_; }
  const V3& clone_position() const { return t0_; }
  const V3& plane() const { return plane_; }
  const V3& plane_anchor_world() const { return anchor_; }
  const Policy& policy() const { return policy_; }
  const MeasurementDiagnostics& last_measurement() const { return diagnostics_; }
  uint64_t accepted_flow_updates() const { return flow_updates_; }
  uint64_t accepted_range_updates() const { return range_updates_; }
  void clear() { initialized_ = false; diagnostics_=MeasurementDiagnostics(); }

  // The seed range is consumed exactly once by this augmentation.  It is not
  // also a range EKF update.  Plane slopes have an explicit independent local
  // prior; no ground height or truth pose enters this construction.
  template<class Native>
  Update initialize_from_range(const Native& native, const RangeRay& ray,
      double range_m, double range_variance_m2, const Eigen::Vector2d& slope_mean,
      const Eigen::Matrix2d& slope_cov,
      const NativeCov& initial_gyro_component = NativeCov::Zero()) {
    Update result;
    if (!native_valid(native) || !covariance_valid(native.cov, true) || !ray.valid() ||
        !std::isfinite(range_m) || range_m < policy_.minimum_depth_m || range_m > policy_.maximum_depth_m ||
        !std::isfinite(range_variance_m2) || range_variance_m2 <= 0. ||
        !slope_mean.allFinite() || slope_mean.cwiseAbs().maxCoeff() > policy_.maximum_abs_slope ||
        !covariance_valid(slope_cov, true) || !covariance_valid(initial_gyro_component) ||
        !covariance_valid(native.cov - initial_gyro_component)) {
      result.reason = "invalid_initialization_or_unknown_covariance"; return result;
    }
    const V3 lever = ray.origin_body + range_m * ray.unit_direction_body;
    const V3 q = native.pos_end + native.rot_end * lever;
    const V3 n(-slope_mean.x(), -slope_mean.y(), 1.);
    const V3 direction = native.rot_end * ray.unit_direction_body;
    if (std::abs(n.dot(direction)) / n.norm() < policy_.minimum_ray_plane_cosine) {
      result.reason = "grazing_seed_ray"; return result;
    }
    Eigen::Matrix<double,kJoint,22> b = Eigen::Matrix<double,kJoint,22>::Zero();
    b.topLeftCorner<kNative,kNative>().setIdentity();
    b.block<6,6>(kClone,0).setIdentity();
    b(kPlane,19) = 1.; b(kPlane+1,20) = 1.;
    b.block<1,3>(kPlane+2,0) = n.transpose() * (-native.rot_end * skew(lever));
    b.block<1,3>(kPlane+2,3) = n.transpose();
    b(kPlane+2,19) = -q.x(); b(kPlane+2,20) = -q.y();
    b(kPlane+2,21) = n.dot(direction);
    Eigen::Matrix<double,22,22> c = Eigen::Matrix<double,22,22>::Zero();
    c.topLeftCorner<kNative,kNative>() = native.cov;
    c.block<2,2>(19,19) = slope_cov; c(21,21) = range_variance_m2;
    JointCov next = b * c * b.transpose();
    JointCov next_pg = b.leftCols<kNative>() * initial_gyro_component * b.leftCols<kNative>().transpose();
    if (!covariance_valid(next) || !covariance_valid(next_pg) || !covariance_valid(next-next_pg)) {
      result.reason = "non_psd_augmentation"; return result;
    }
    p_ = symmetric(next); pg_ = symmetric(next_pg);
    r0_ = native.rot_end; t0_ = native.pos_end; r1_ = native.rot_end; t1_ = native.pos_end;
    plane_ = V3(slope_mean.x(), slope_mean.y(), n.dot(q)); anchor_ = q;
    initialized_ = true; result.accepted = true; result.reason = "range_anchored_joint_augmentation";
    return result;
  }

  // F and Q must be the actual native 19D propagation, including every IMU
  // substep.  Qgyro is the separately accumulated gyro/bias process component,
  // not the whole process covariance.  The native top-left covariance is checked.
  template<class Native>
  Update propagate_native(const Native& after, const NativeCov& f,
                          const NativeCov& q, const NativeCov& qgyro = NativeCov::Zero()) {
    Update result;
    if (!initialized_ || !native_valid(after) || !f.allFinite() || !covariance_valid(q) ||
        !covariance_valid(qgyro) || !covariance_valid(q-qgyro)) {
      result.reason = "invalid_native_propagation"; return result;
    }
    JointCov phi = JointCov::Identity(); phi.topLeftCorner<kNative,kNative>() = f;
    JointCov next = phi * p_ * phi.transpose(); next.topLeftCorner<kNative,kNative>() += q;
    JointCov next_pg = phi * pg_ * phi.transpose(); next_pg.topLeftCorner<kNative,kNative>() += qgyro;
    if (!native_matches(next.topLeftCorner<kNative,kNative>(), after.cov) ||
        !covariance_valid(next) || !covariance_valid(next_pg) || !covariance_valid(next-next_pg)) {
      result.reason = "native_propagation_covariance_mismatch"; return result;
    }
    p_ = symmetric(next); pg_ = symmetric(next_pg); r1_ = after.rot_end; t1_ = after.pos_end;
    result.accepted = true; result.reason = "full_cross_covariance_propagated"; return result;
  }

  // Extends an existing native update by its Gaussian conditional distribution.
  // Assumes the update is an independent measurement of the native state with a
  // fixed map.  This cannot restore omitted historical map correlations.
  // Input after.cov is BEFORE injection reset.  This method resets both active
  // and clone SO(3) error coordinates and writes the resulting Pxx back to after.
  template<class Native>
  Update adopt_native_update(const Native& before, Native& after) {
    Update result;
    if (!initialized_ || !native_valid(before) || !native_valid(after) ||
        !native_matches(p_.topLeftCorner<kNative,kNative>(), before.cov) ||
        (before.rot_end-r1_).norm() > 1e-7 || (before.pos_end-t1_).norm() > 1e-7 ||
        !covariance_valid(after.cov, true) || !covariance_valid(before.cov-after.cov)) {
      result.reason = "invalid_or_noncontractive_native_update"; return result;
    }
    Eigen::LDLT<NativeCov> factor(before.cov);
    if (factor.info() != Eigen::Success || !factor.isPositive()) {
      result.reason = "native_prior_not_invertible"; return result;
    }
    const Eigen::Matrix<double,9,kNative> conditional =
      factor.solve(p_.topRightCorner<kNative,9>()).transpose();
    const NativeCov a = factor.solve(after.cov.transpose()).transpose();
    result.delta.head<kNative>() = difference(after, before);
    result.delta.tail<9>() = conditional * result.delta.head<kNative>();
    JointCov next = p_;
    next.topLeftCorner<kNative,kNative>() = after.cov;
    next.topRightCorner<kNative,9>() = after.cov * conditional.transpose();
    next.bottomLeftCorner<9,kNative>() = next.topRightCorner<kNative,9>().transpose();
    next.bottomRightCorner<9,9>() -= conditional * (before.cov-after.cov) * conditional.transpose();
    JointCov l = JointCov::Identity(); l.topLeftCorner<kNative,kNative>() = a;
    l.bottomLeftCorner<9,kNative>() = conditional * (a-NativeCov::Identity());
    JointCov next_pg = l * pg_ * l.transpose();
    JointCov reset = reset_matrix(result.delta);
    next = reset * next * reset.transpose(); next_pg = reset * next_pg * reset.transpose();
    const V3 next_plane = plane_ + result.delta.tail<3>();
    if (!covariance_valid(next) || !covariance_valid(next_pg) || !covariance_valid(next-next_pg) ||
        !next_plane.allFinite() || next_plane.head<2>().cwiseAbs().maxCoeff() > policy_.maximum_abs_slope) {
      result.reason = "invalid_native_conditional_or_plane"; return result;
    }
    r0_ = r0_ * exp_so3(result.delta.segment<3>(kClone));
    t0_ += result.delta.segment<3>(kClone+3); plane_ = next_plane;
    p_ = symmetric(next); pg_ = symmetric(next_pg); r1_ = after.rot_end; t1_ = after.pos_end;
    after.cov = p_.topLeftCorner<kNative,kNative>();
    result.accepted = true; result.reason = "native_conditional_update_and_so3_reset";
    result.native_delta_norm = result.delta.head<kNative>().norm();
    result.covariance_delta_norm = (after.cov-before.cov).norm(); return result;
  }

  Projection project(const Camera& camera, const Eigen::Vector2d& first_pixel) const {
    Projection out;
    if (!initialized_ || !camera.valid() || !first_pixel.allFinite()) {
      out.reason = "invalid_camera_or_uninitialized"; return out;
    }
    const V3 ray((first_pixel.x()-camera.cx)/camera.fx,
                 (first_pixel.y()-camera.cy)/camera.fy, 1.);
    const V3 origin = t0_ + r0_ * camera.optical_origin_body;
    const V3 direction = r0_ * camera.body_from_optical * ray;
    const V3 n(-plane_.x(),-plane_.y(),1.);
    const double denominator = n.dot(direction);
    if (std::abs(denominator)/(n.norm()*direction.norm()) < policy_.minimum_ray_plane_cosine) {
      out.reason = "grazing_camera_ray"; return out;
    }
    const double depth = (plane_.z()-n.dot(origin))/denominator;
    if (!std::isfinite(depth) || depth < policy_.minimum_depth_m || depth > policy_.maximum_depth_m) {
      out.reason = "invalid_initial_depth"; return out;
    }
    const V3 point = origin + depth * direction;
    if ((point-anchor_).head<2>().norm() > policy_.maximum_local_radius_m) {
      out.reason = "outside_local_plane_domain"; return out;
    }
    const M3 rcw = camera.body_from_optical.transpose() * r1_.transpose();
    const V3 point_body = r1_.transpose()*(point-t1_);
    const V3 c = camera.body_from_optical.transpose()*(point_body-camera.optical_origin_body);
    if (!c.allFinite() || c.z() < policy_.minimum_depth_m || c.z() > policy_.maximum_depth_m) {
      out.reason = "invalid_final_depth"; return out;
    }
    Eigen::Matrix<double,2,3> j;
    j << camera.fx/c.z(),0.,-camera.fx*c.x()/(c.z()*c.z()),
         0.,camera.fy/c.z(),-camera.fy*c.y()/(c.z()*c.z());
    const M3 plane_projector = M3::Identity()-direction*n.transpose()/denominator;
    out.pixel = Eigen::Vector2d(camera.fx*c.x()/c.z()+camera.cx,camera.fy*c.y()/c.z()+camera.cy);
    out.jacobian.block<2,3>(0,0) = j*camera.body_from_optical.transpose()*skew(point_body);
    out.jacobian.block<2,3>(0,3) = -j*rcw;
    out.jacobian.block<2,3>(0,kClone) =
      -j*rcw*plane_projector*r0_*skew(camera.optical_origin_body+depth*camera.body_from_optical*ray);
    out.jacobian.block<2,3>(0,kClone+3) = j*rcw*plane_projector;
    out.jacobian.col(kPlane) = j*rcw*direction*(point.x()/denominator);
    out.jacobian.col(kPlane+1) = j*rcw*direction*(point.y()/denominator);
    out.jacobian.col(kPlane+2) = j*rcw*direction/denominator;
    Eigen::Matrix<double,3,2> ray_pixel = Eigen::Matrix<double,3,2>::Zero();
    ray_pixel(0,0)=1./camera.fx; ray_pixel(1,1)=1./camera.fy;
    out.first_pixel_jacobian = j*rcw*depth*plane_projector*r0_*camera.body_from_optical*ray_pixel;
    out.initial_optical_depth_m=depth; out.final_optical_depth_m=c.z(); out.point_world=point;
    out.valid = out.jacobian.allFinite() && out.first_pixel_jacobian.allFinite();
    out.reason = out.valid ? "valid_local_plane_projection" : "nonfinite_projection"; return out;
  }

  template<class Native>
  Update update_range(Native& state, const RangeRay& ray, double measured_m, double variance_m2) {
    diagnostics_=MeasurementDiagnostics(); diagnostics_.kind="range";
    Update out;
    if (!ready_state(state) || !ray.valid() || !std::isfinite(measured_m) ||
        measured_m < policy_.minimum_depth_m || measured_m > policy_.maximum_depth_m ||
        !std::isfinite(variance_m2) || variance_m2 <= 0.) {
      out.reason="invalid_range_or_unknown_covariance"; return out;
    }
    const V3 n(-plane_.x(),-plane_.y(),1.);
    const V3 origin=state.pos_end+state.rot_end*ray.origin_body;
    const V3 direction=state.rot_end*ray.unit_direction_body;
    const double denominator=n.dot(direction);
    if (std::abs(denominator)/n.norm() < policy_.minimum_ray_plane_cosine) {
      out.reason="grazing_range_ray"; return out;
    }
    const double rho=(plane_.z()-n.dot(origin))/denominator;
    const V3 q=origin+rho*direction;
    if (!std::isfinite(rho) || rho < policy_.minimum_depth_m || rho > policy_.maximum_depth_m ||
        (q-anchor_).head<2>().norm() > policy_.maximum_local_radius_m) {
      out.reason="range_outside_local_plane_domain"; return out;
    }
    Eigen::MatrixXd h=Eigen::MatrixXd::Zero(1,kJoint);
    h.block<1,3>(0,0)=n.transpose()*state.rot_end*skew(ray.origin_body+rho*ray.unit_direction_body)/denominator;
    h.block<1,3>(0,3)=-n.transpose()/denominator;
    h(0,kPlane)=q.x()/denominator; h(0,kPlane+1)=q.y()/denominator; h(0,kPlane+2)=1./denominator;
    Eigen::VectorXd residual(1); residual(0)=measured_m-rho;
    Eigen::MatrixXd noise(1,1); noise(0,0)=variance_m2;
    out=measurement_update(state,h,residual,noise,policy_.range_nis_limit);
    if (out.accepted) ++range_updates_;
    return out;
  }

  template<class Native>
  Update update_tracks(Native& state, const Camera& camera, const std::vector<Track>& tracks) {
    diagnostics_=MeasurementDiagnostics(); diagnostics_.kind="raw_two_view_pixels";
    Update out; out.input_tracks=tracks.size();
    if (!ready_state(state) || !camera.valid()) { out.reason="invalid_flow_state_or_camera"; return out; }
    if (tracks.size()<policy_.minimum_tracks || tracks.size()>policy_.maximum_tracks) {
      out.reason="insufficient_or_excessive_tracks"; return out;
    }
    if ((state.pos_end-t0_).norm()<policy_.minimum_translation_m) {
      out.reason="pure_rotation_or_insufficient_translation"; return out;
    }
    std::vector<Projection> predictions; std::vector<Eigen::Matrix2d> noises;
    std::vector<const Track*> selected;
    std::set<uint64_t> identities;
    for (const Track& track:tracks) if (!identities.insert(track.id).second) {
      out.reason="duplicate_track_identity"; return out;
    }
    Eigen::Vector2d lower=Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity());
    Eigen::Vector2d upper=-lower;
    for (const Track& track : tracks) {
      if (!track.first_pixel.allFinite() || !track.second_pixel.allFinite() ||
          !covariance_valid(track.first_cov,true) || !covariance_valid(track.second_cov,true)) {
        ++out.rejected_tracks; continue;
      }
      Projection prediction=project(camera,track.first_pixel);
      if (!prediction.valid) { ++out.rejected_tracks; continue; }
      Eigen::Matrix2d noise=prediction.first_pixel_jacobian*track.first_cov*
        prediction.first_pixel_jacobian.transpose()+track.second_cov;
      const Eigen::Vector2d residual=track.second_pixel-prediction.pixel;
      const Eigen::Matrix2d innovation=prediction.jacobian*p_*prediction.jacobian.transpose()+noise;
      Eigen::LDLT<Eigen::Matrix2d> factor(innovation);
      const double nis=residual.dot(factor.solve(residual));
      if (factor.info()!=Eigen::Success || !factor.isPositive() || !std::isfinite(nis) || nis>policy_.individual_nis_limit) {
        ++out.rejected_tracks; continue;
      }
      selected.push_back(&track); predictions.push_back(prediction); noises.push_back(noise);
      lower=lower.cwiseMin(track.first_pixel); upper=upper.cwiseMax(track.first_pixel);
    }
    out.accepted_tracks=selected.size();
    if (selected.size()<policy_.minimum_tracks) { out.reason="insufficient_inlier_tracks"; return out; }
    if ((upper-lower).minCoeff()<policy_.minimum_pixel_span) {
      out.reason="insufficient_spatial_texture_support"; return out;
    }
    const int rows=static_cast<int>(2*selected.size());
    Eigen::MatrixXd h(rows,kJoint),noise=Eigen::MatrixXd::Zero(rows,rows);
    Eigen::VectorXd residual(rows);
    M3 information=M3::Zero();
    for (size_t i=0;i<selected.size();++i) {
      const int row=static_cast<int>(2*i);
      h.block(row,0,2,kJoint)=predictions[i].jacobian;
      residual.segment<2>(row)=selected[i]->second_pixel-predictions[i].pixel;
      noise.block<2,2>(row,row)=noises[i];
      const Eigen::Matrix<double,2,3> ht=predictions[i].jacobian.block<2,3>(0,3);
      information+=ht.transpose()*noises[i].ldlt().solve(ht);
    }
    Eigen::SelfAdjointEigenSolver<M3> eig(information,Eigen::EigenvaluesOnly);
    if (eig.info()!=Eigen::Success || eig.eigenvalues().minCoeff()<policy_.minimum_translation_information) {
      out.reason="translation_information_degenerate"; return out;
    }
    Update applied=measurement_update(state,h,residual,noise,policy_.maximum_batch_nis_per_dof*rows);
    applied.input_tracks=out.input_tracks; applied.accepted_tracks=out.accepted_tracks;
    applied.rejected_tracks=out.rejected_tracks;
    if (applied.accepted) {
      ++flow_updates_;
      for (size_t i=0;i<selected.size();++i) {
        diagnostics_.accepted_track_ids.push_back(selected[i]->id);
        diagnostics_.predicted_pixels.push_back(predictions[i].pixel);
      }
    }
    return applied;
  }

  // Deliberately misspecified controls.  Preserve marginals and remove only the
  // declared cross component.  Apply the same block partition to the tagged
  // component, preserving a PSD decomposition for subsequent propagation.
  // An indefinite result is rejected, not repaired by arbitrary jitter.
  bool apply_cross_ablation(CrossAblation mode) {
    if (!initialized_) return false;
    if (mode==CrossAblation::None) return true;
    JointCov next=p_,next_pg=pg_;
    if (mode==CrossAblation::ZeroGyroCross) {
      for (int i=0;i<kJoint;++i) for (int j=0;j<kJoint;++j)
        if ((i<kNative)!=(j<kNative) || (i>=kClone && i<kPlane)!=(j>=kClone && j<kPlane))
        {
          next(i,j)-=pg_(i,j); next_pg(i,j)=0.;
        }
    } else {
      next.topRightCorner<kPlane,3>().setZero();
      next.bottomLeftCorner<3,kPlane>().setZero();
      next_pg.topRightCorner<kPlane,3>().setZero();
      next_pg.bottomLeftCorner<3,kPlane>().setZero();
    }
    if (!covariance_valid(next) || !covariance_valid(next_pg) || !covariance_valid(next-next_pg)) return false;
    p_=symmetric(next); pg_=symmetric(next_pg); return true;
  }

  // Public diagnostic perturbation for independent central-difference checks.
  // This changes only nominal variables; it does not perform an EKF update.
  void diagnostic_perturb(const JointDelta& delta) {
    if (!delta.allFinite()) throw std::invalid_argument("nonfinite perturbation");
    r1_=r1_*exp_so3(delta.segment<3>(0)); t1_+=delta.segment<3>(3);
    r0_=r0_*exp_so3(delta.segment<3>(kClone)); t0_+=delta.segment<3>(kClone+3);
    plane_+=delta.tail<3>();
  }

 private:
  template<class Native> static bool native_valid(const Native& s) {
    return rotation_valid(s.rot_end) && s.pos_end.allFinite() && s.vel_end.allFinite() &&
      s.bias_g.allFinite() && s.bias_a.allFinite() && s.gravity.allFinite() &&
      std::isfinite(s.inv_expo_time) && covariance_valid(s.cov);
  }
  template<class Native> bool ready_state(const Native& state) const {
    return initialized_ && native_valid(state) && native_matches(p_.topLeftCorner<kNative,kNative>(),state.cov) &&
      (state.rot_end-r1_).norm()<1e-7 && (state.pos_end-t1_).norm()<1e-7;
  }
  static bool native_matches(const NativeCov& a,const NativeCov& b) {
    return a.allFinite() && b.allFinite() && (a-b).norm()<=1e-7*std::max(1.,b.norm());
  }
  static JointCov symmetric(const JointCov& p) { return .5*(p+p.transpose()); }
  static JointCov reset_matrix(const JointDelta& dx) {
    JointCov reset=JointCov::Identity();
    reset.topLeftCorner<3,3>()=right_reset(dx.head<3>());
    reset.block<3,3>(kClone,kClone)=right_reset(dx.segment<3>(kClone)); return reset;
  }
  template<class Native> static NativeDelta difference(const Native& a,const Native& b) {
    NativeDelta d; d.head<3>()=log_so3(b.rot_end.transpose()*a.rot_end);
    d.segment<3>(3)=a.pos_end-b.pos_end; d(6)=a.inv_expo_time-b.inv_expo_time;
    d.segment<3>(7)=a.vel_end-b.vel_end; d.segment<3>(10)=a.bias_g-b.bias_g;
    d.segment<3>(13)=a.bias_a-b.bias_a; d.segment<3>(16)=a.gravity-b.gravity; return d;
  }
  template<class Native> static void inject_native(Native& s,const NativeDelta& d) {
    s.rot_end=s.rot_end*exp_so3(d.head<3>()); s.pos_end+=d.segment<3>(3);
    s.inv_expo_time+=d(6); s.vel_end+=d.segment<3>(7); s.bias_g+=d.segment<3>(10);
    s.bias_a+=d.segment<3>(13); s.gravity+=d.segment<3>(16);
  }
  template<class Native> Update measurement_update(Native& state,const Eigen::MatrixXd& h,
      const Eigen::VectorXd& residual,const Eigen::MatrixXd& noise,double nis_limit) {
    Update out; out.dof=static_cast<int>(residual.size());
    if (!h.allFinite() || !residual.allFinite() || !covariance_valid(noise,true)) {
      out.reason="invalid_measurement_covariance"; return out;
    }
    const Eigen::MatrixXd innovation=h*p_*h.transpose()+noise;
    Eigen::LDLT<Eigen::MatrixXd> factor(innovation);
    if (factor.info()!=Eigen::Success || !factor.isPositive()) {
      out.reason="nonpositive_innovation"; return out;
    }
    out.nis=residual.dot(factor.solve(residual));
    if (!std::isfinite(out.nis) || out.nis>nis_limit) { out.reason="innovation_gate"; return out; }
    const Eigen::MatrixXd gain=factor.solve(h*p_).transpose();
    out.delta=gain*residual;
    if (!out.delta.allFinite() || out.delta.head<3>().norm()>policy_.maximum_rotation_correction_rad ||
        out.delta.segment<3>(3).norm()>policy_.maximum_position_correction_m ||
        out.delta.segment<3>(kClone).norm()>policy_.maximum_rotation_correction_rad ||
        out.delta.segment<3>(kClone+3).norm()>policy_.maximum_position_correction_m) {
      out.reason="correction_outside_linearization_domain"; return out;
    }
    const V3 next_plane=plane_+out.delta.tail<3>();
    if (!next_plane.allFinite() || next_plane.head<2>().cwiseAbs().maxCoeff()>policy_.maximum_abs_slope) {
      out.reason="plane_slope_outside_local_model"; return out;
    }
    const JointCov l=JointCov::Identity()-gain*h;
    const JointCov reset=reset_matrix(out.delta);
    JointCov next=reset*(l*p_*l.transpose()+gain*noise*gain.transpose())*reset.transpose();
    JointCov next_pg=reset*l*pg_*l.transpose()*reset.transpose();
    if (!covariance_valid(next) || !covariance_valid(next_pg) || !covariance_valid(next-next_pg)) {
      out.reason="non_psd_posterior"; return out;
    }
    out.native_delta_norm=out.delta.head<kNative>().norm();
    out.covariance_delta_norm=(next-p_).norm();
    inject_native(state,out.delta.head<kNative>());
    r0_=r0_*exp_so3(out.delta.segment<3>(kClone)); t0_+=out.delta.segment<3>(kClone+3);
    plane_=next_plane; p_=symmetric(next); pg_=symmetric(next_pg);
    state.cov=p_.topLeftCorner<kNative,kNative>(); r1_=state.rot_end; t1_=state.pos_end;
    diagnostics_.applied=true; diagnostics_.jacobian=h; diagnostics_.noise=noise;
    diagnostics_.innovation=innovation; diagnostics_.gain=gain; diagnostics_.residual=residual;
    out.accepted=true; out.reason="joint_nonlinear_update_applied"; return out;
  }
  Policy policy_;
  bool initialized_=false;
  JointCov p_=JointCov::Zero(),pg_=JointCov::Zero();
  M3 r0_=M3::Identity(),r1_=M3::Identity();
  V3 t0_=V3::Zero(),t1_=V3::Zero(),plane_=V3::Zero(),anchor_=V3::Zero();
  uint64_t flow_updates_=0,range_updates_=0;
  MeasurementDiagnostics diagnostics_;
};
} // namespace impact_research
#endif
