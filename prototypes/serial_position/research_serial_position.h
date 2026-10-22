// Two-stage engineering baseline: robust geometric position fit followed by a
// native 19D position-measurement EKF. Shared estimator/measurement correlation
// is deliberately ignored at the serial fusion boundary and explicitly logged.
#ifndef IMPACT_RESEARCH_SERIAL_POSITION_H
#define IMPACT_RESEARCH_SERIAL_POSITION_H
#include "research_joint_flow.h"
#include <chrono>
#include <set>

namespace impact_research {
struct SerialPolicy {
  int maximum_iterations=12,maximum_line_search=8;
  double huber_sigma=2.5,final_inlier_nis=9.210340371976184;
  double maximum_information_condition=1e8,minimum_information_eigenvalue=1e-8;
  double step_tolerance_m=1e-6,relative_objective_tolerance=1e-8;
  double maximum_fit_offset_m=.5,native_position_nis_limit=11.344866730144373;
  bool valid() const {
    const double x[]={huber_sigma,final_inlier_nis,maximum_information_condition,
      minimum_information_eigenvalue,step_tolerance_m,relative_objective_tolerance,
      maximum_fit_offset_m,native_position_nis_limit};
    for(double a:x)if(!std::isfinite(a)||a<=0.)return false;
    return maximum_iterations>0&&maximum_iterations<=30&&maximum_line_search>0&&maximum_line_search<=16;
  }
};
struct SerialFit {
  bool accepted=false;
  std::string reason;
  V3 position=V3::Zero();
  M3 covariance=M3::Zero(),pixel_covariance_part=M3::Zero(),nuisance_covariance_part=M3::Zero();
  Eigen::MatrixXd influence,position_jacobian,nuisance_jacobian,nuisance_covariance,pixel_covariance,weights;
  Eigen::VectorXd residual;
  std::vector<uint64_t> accepted_track_ids;
  size_t input_tracks=0;
  int iterations=0,line_search_evaluations=0;
  double robust_objective=std::numeric_limits<double>::quiet_NaN();
  double information_condition=std::numeric_limits<double>::quiet_NaN();
  double elapsed_ms=0.;
};
struct SerialApplied {
  Update update;
  Eigen::Matrix<double,3,kNative> native_h=Eigen::Matrix<double,3,kNative>::Zero();
  Eigen::Matrix<double,kNative,3> native_gain=Eigen::Matrix<double,kNative,3>::Zero();
  V3 position_innovation=V3::Zero();
  M3 position_innovation_covariance=M3::Zero();
};
namespace serial_detail {
inline Eigen::Matrix<double,12,12> nuisance_marginal(const JointFlow& joint) {
  Eigen::Matrix<double,12,kJoint> select=Eigen::Matrix<double,12,kJoint>::Zero();
  select.topLeftCorner<3,3>().setIdentity();select.block<9,9>(3,kClone).setIdentity();
  return select*joint.covariance()*select.transpose();
}
struct Evaluation {
  bool valid=false;
  double cost=0.;
  M3 information=M3::Zero();
  V3 gradient=V3::Zero();
  std::vector<Projection> predictions;
  std::vector<Eigen::Matrix2d> noises;
  std::vector<double> robust_weights;
  std::vector<Eigen::Vector2d> residuals;
};
inline Evaluation evaluate(const JointFlow& base,const Camera& camera,
    const std::vector<const Track*>& tracks,const V3& original,const V3& position,
    const SerialPolicy& policy) {
  Evaluation out;JointFlow model=base;JointDelta shift=JointDelta::Zero();
  shift.segment<3>(3)=position-original;model.diagnostic_perturb(shift);
  for(const Track* track:tracks) {
    const Projection projection=model.project(camera,track->first_pixel);
    if(!projection.valid)return out;
    const Eigen::Matrix2d r=projection.first_pixel_jacobian*track->first_cov*
      projection.first_pixel_jacobian.transpose()+track->second_cov;
    Eigen::LDLT<Eigen::Matrix2d> factor(r);
    if(factor.info()!=Eigen::Success||!factor.isPositive())return out;
    const Eigen::Vector2d residual=track->second_pixel-projection.pixel;
    const double squared=residual.dot(factor.solve(residual));
    if(!std::isfinite(squared)||squared<0.)return out;
    const double norm=std::sqrt(squared),weight=norm>policy.huber_sigma?policy.huber_sigma/norm:1.;
    const auto j=projection.jacobian.block<2,3>(0,3);
    out.information+=weight*j.transpose()*factor.solve(j);
    out.gradient+=weight*j.transpose()*factor.solve(residual);
    out.cost+=norm<=policy.huber_sigma?.5*squared:policy.huber_sigma*(norm-.5*policy.huber_sigma);
    out.predictions.push_back(projection);out.noises.push_back(r);
    out.robust_weights.push_back(weight);out.residuals.push_back(residual);
  }
  out.valid=std::isfinite(out.cost)&&out.information.allFinite()&&out.gradient.allFinite();return out;
}
inline bool conditioned(const M3& information,const SerialPolicy& policy,double& condition) {
  Eigen::SelfAdjointEigenSolver<M3> solver(information,Eigen::EigenvaluesOnly);
  if(solver.info()!=Eigen::Success||solver.eigenvalues().minCoeff()<policy.minimum_information_eigenvalue)return false;
  condition=solver.eigenvalues().maxCoeff()/solver.eigenvalues().minCoeff();
  return std::isfinite(condition)&&condition<=policy.maximum_information_condition;
}
}

// Stage one uses pixels only to fit position, at fixed nuisance means.  Its
// covariance then propagates the complete 12D nuisance marginal (active attitude
// 3, clone pose 6, local plane 3), including all their mutual cross blocks.
// No nuisance is treated as calibrated truth.  Correlation of this derived
// position measurement with the native state is the serial approximation.
template<class Native>
SerialFit fit_serial_position(const JointFlow& joint,const Native& native,const Camera& camera,
    const std::vector<Track>& tracks,const SerialPolicy& policy=SerialPolicy()) {
  SerialFit out;out.input_tracks=tracks.size();const auto started=std::chrono::steady_clock::now();
  auto finish=[&](const std::string& reason,bool accepted=false) {
    out.reason=reason;out.accepted=accepted;
    out.elapsed_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
    return out;
  };
  if(!policy.valid()||!joint.initialized()||!camera.valid()||!native.pos_end.allFinite()||
      !rotation_valid(native.rot_end)||!covariance_valid(joint.covariance()))return finish("invalid_serial_contract");
  const Policy& common=joint.policy();
  if(tracks.size()<common.minimum_tracks||tracks.size()>common.maximum_tracks)return finish("insufficient_or_excessive_tracks");
  if((native.pos_end-joint.clone_position()).norm()<common.minimum_translation_m)
    return finish("pure_rotation_or_insufficient_translation");
  std::set<uint64_t> ids;std::vector<const Track*> selected;
  Eigen::Vector2d lower=Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity()),upper=-lower;
  for(const Track& t:tracks) {
    if(!ids.insert(t.id).second)return finish("duplicate_track_identity");
    if(!t.first_pixel.allFinite()||!t.second_pixel.allFinite()||!covariance_valid(t.first_cov,true)||
       !covariance_valid(t.second_cov,true))continue;
    const auto projection=joint.project(camera,t.first_pixel);
    if(!projection.valid)continue;
    selected.push_back(&t);lower=lower.cwiseMin(t.first_pixel);upper=upper.cwiseMax(t.first_pixel);
  }
  if(selected.size()<common.minimum_tracks)return finish("insufficient_valid_tracks");
  if((upper-lower).minCoeff()<common.minimum_pixel_span)return finish("insufficient_spatial_texture_support");
  V3 position=native.pos_end;
  serial_detail::Evaluation current;
  bool final_trim=false,converged=false;
  for(int iteration=0;iteration<policy.maximum_iterations;++iteration) {
    ++out.iterations;
    current=serial_detail::evaluate(joint,camera,selected,native.pos_end,position,policy);
    if(!current.valid)return finish("position_fit_invalid_geometry");
    if(!serial_detail::conditioned(current.information,policy,out.information_condition))
      return finish("position_fit_unobservable_or_ill_conditioned");
    const V3 increment=current.information.ldlt().solve(current.gradient);
    if(!increment.allFinite())return finish("position_fit_nonfinite_step");
    bool accepted=false;double relative_decrease=0.;
    if(increment.norm()<=policy.step_tolerance_m)accepted=true;
    else {
      double scale=1.;
      for(int trial=0;trial<policy.maximum_line_search;++trial,scale*=.5) {
        ++out.line_search_evaluations;const V3 candidate=position+scale*increment;
        if((candidate-native.pos_end).norm()>policy.maximum_fit_offset_m)continue;
        auto proposed=serial_detail::evaluate(joint,camera,selected,native.pos_end,candidate,policy);
        if(proposed.valid&&proposed.cost<current.cost) {
          relative_decrease=(current.cost-proposed.cost)/std::max(1.,current.cost);
          position=candidate;current=std::move(proposed);accepted=true;break;
        }
      }
    }
    if(!accepted)return finish("position_fit_line_search_failed");
    const bool locally_converged=increment.norm()<=policy.step_tolerance_m||
      (relative_decrease>0.&&relative_decrease<=policy.relative_objective_tolerance);
    if(!locally_converged)continue;
    if(!final_trim) {
      // One declared residual-based trim, then refit within the SAME iteration
      // budget. This is data-dependent robust estimation, not exact Gaussian ML.
      std::vector<const Track*> inliers;
      const auto pnn=serial_detail::nuisance_marginal(joint);
      for(size_t i=0;i<selected.size();++i) {
        const auto& r=current.residuals[i];
        Eigen::Matrix<double,2,12> hn;
        hn.leftCols<3>()=current.predictions[i].jacobian.block<2,3>(0,0);
        hn.rightCols<9>()=current.predictions[i].jacobian.block<2,9>(0,kClone);
        // The residual trim allows explicitly uncertain nuisance means.  This
        // conservative marginal gate is an engineering selection rule after
        // fitting, not a claim of independent chi-square calibrated residuals.
        const Eigen::Matrix2d marginal=current.noises[i]+hn*pnn*hn.transpose();
        if(r.dot(marginal.ldlt().solve(r))<=policy.final_inlier_nis)inliers.push_back(selected[i]);
      }
      if(inliers.size()<common.minimum_tracks)return finish("insufficient_robust_inliers");
      final_trim=true;
      if(inliers.size()!=selected.size()) {selected=std::move(inliers);continue;}
    }
    converged=true;break;
  }
  if(!converged)return finish("position_fit_iteration_budget_exhausted");
  current=serial_detail::evaluate(joint,camera,selected,native.pos_end,position,policy);
  if(!current.valid||!serial_detail::conditioned(current.information,policy,out.information_condition))
    return finish("invalid_final_position_information");
  lower.setConstant(std::numeric_limits<double>::infinity());upper=-lower;
  for(const Track* track:selected) {lower=lower.cwiseMin(track->first_pixel);upper=upper.cwiseMax(track->first_pixel);}
  if((upper-lower).minCoeff()<common.minimum_pixel_span)return finish("insufficient_final_spatial_support");
  const int count=static_cast<int>(selected.size()),rows=2*count;
  Eigen::MatrixXd jp(rows,3),hn(rows,12),r=Eigen::MatrixXd::Zero(rows,rows),w=Eigen::MatrixXd::Zero(rows,rows);
  Eigen::VectorXd residual(rows);
  for(int i=0;i<count;++i) {
    const auto& projection=current.predictions[i];
    jp.block<2,3>(2*i,0)=projection.jacobian.block<2,3>(0,3);
    hn.block<2,3>(2*i,0)=projection.jacobian.block<2,3>(0,0);
    hn.block<2,9>(2*i,3)=projection.jacobian.block<2,9>(0,kClone);
    r.block<2,2>(2*i,2*i)=current.noises[i];
    w.block<2,2>(2*i,2*i)=current.robust_weights[i]*current.noises[i].inverse();
    residual.segment<2>(2*i)=current.residuals[i];out.accepted_track_ids.push_back(selected[i]->id);
  }
  const Eigen::Matrix<double,12,12> pnn=serial_detail::nuisance_marginal(joint);
  const Eigen::MatrixXd influence=current.information.ldlt().solve(jp.transpose()*w);
  const Eigen::MatrixXd nuisance_influence=-influence*hn;
  out.pixel_covariance_part=influence*r*influence.transpose();
  out.nuisance_covariance_part=nuisance_influence*pnn*nuisance_influence.transpose();
  out.covariance=out.pixel_covariance_part+out.nuisance_covariance_part;
  out.covariance=.5*(out.covariance+out.covariance.transpose()).eval();
  if(!covariance_valid(out.covariance,true)||!covariance_valid(out.nuisance_covariance_part))
    return finish("invalid_propagated_serial_covariance");
  out.position=position;out.robust_objective=current.cost;
  out.influence=influence;out.position_jacobian=jp;out.nuisance_jacobian=hn;
  out.nuisance_covariance=pnn;out.pixel_covariance=r;out.weights=w;out.residual=residual;
  return finish("robust_geometry_position_fit_with_uncertain_nuisance",true);
}

template<class Native>
SerialApplied apply_serial_position(JointFlow& joint,Native& native,const SerialFit& fit,
    const SerialPolicy& policy=SerialPolicy()) {
  SerialApplied out;
  if(!fit.accepted||!fit.position.allFinite()||!covariance_valid(fit.covariance,true)||
     !covariance_valid(native.cov,true)||!policy.valid()) {
    out.update.reason="invalid_serial_position_measurement";return out;
  }
  out.native_h.block<3,3>(0,3).setIdentity();
  out.position_innovation=fit.position-native.pos_end;
  out.position_innovation_covariance=out.native_h*native.cov*out.native_h.transpose()+fit.covariance;
  Eigen::LDLT<M3> factor(out.position_innovation_covariance);
  if(factor.info()!=Eigen::Success||!factor.isPositive()) {out.update.reason="invalid_serial_native_innovation";return out;}
  const double nis=out.position_innovation.dot(factor.solve(out.position_innovation));
  if(!std::isfinite(nis)||nis>policy.native_position_nis_limit) {out.update.reason="serial_native_position_NIS_gate";out.update.nis=nis;return out;}
  out.native_gain=factor.solve(out.native_h*native.cov).transpose();
  const NativeDelta dx=out.native_gain*out.position_innovation;
  if(!dx.allFinite()||dx.head<3>().norm()>joint.policy().maximum_rotation_correction_rad||
     dx.segment<3>(3).norm()>joint.policy().maximum_position_correction_m) {
    out.update.reason="serial_correction_outside_linearization_domain";return out;
  }
  const Native before=native;Native proposed=native;
  proposed.rot_end=proposed.rot_end*exp_so3(dx.head<3>());proposed.pos_end+=dx.segment<3>(3);
  proposed.inv_expo_time+=dx(6);proposed.vel_end+=dx.segment<3>(7);proposed.bias_g+=dx.segment<3>(10);
  proposed.bias_a+=dx.segment<3>(13);proposed.gravity+=dx.segment<3>(16);
  const NativeCov l=NativeCov::Identity()-out.native_gain*out.native_h;
  proposed.cov=l*before.cov*l.transpose()+out.native_gain*fit.covariance*out.native_gain.transpose();
  proposed.cov=.5*(proposed.cov+proposed.cov.transpose()).eval();
  // Same conditional auxiliary update and SO(3) reset as the no-flow control.
  // There is deliberately NO joint raw-pixel Kalman gain in this baseline.
  out.update=joint.adopt_native_update(before,proposed);
  out.update.nis=nis;out.update.dof=3;out.update.input_tracks=fit.input_tracks;
  out.update.accepted_tracks=fit.accepted_track_ids.size();
  out.update.rejected_tracks=fit.input_tracks-fit.accepted_track_ids.size();
  if(out.update.accepted) {native=proposed;out.update.reason="serial_position_measurement_native_update_applied";}
  return out;
}
} // namespace impact_research
#endif
