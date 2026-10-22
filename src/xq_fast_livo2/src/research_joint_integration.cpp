#include "research_joint_integration.h"
#include "common_lib.h"
#include "research_joint_flow.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <stdexcept>

namespace impact_research {
namespace {
void number(std::ostream& out, double x) {
  if (std::isfinite(x)) out << std::setprecision(17) << x; else out << "null";
}
template<class D> void matrix(std::ostream& out, const Eigen::MatrixBase<D>& x) {
  out << '[';
  for (Eigen::Index i=0;i<x.rows();++i) {
    if (i) out << ',';
    out << '[';
    for (Eigen::Index j=0;j<x.cols();++j) { if (j) out << ','; number(out,x(i,j)); }
    out << ']';
  }
  out << ']';
}
// All messages below are implementation-owned fixed tokens, not user strings.
void token(std::ostream& out,const std::string& x) {
  out << '"'; for (unsigned char c:x) {
    if (c=='"'||c=='\\') out << '\\';
    if (c<32) throw std::invalid_argument("control character in research token");
    out << c;
  } out << '"';
}
void state_json(std::ostream& out,const StatesGroup& s) {
  out << "{\"R\":";matrix(out,s.rot_end);out << ",\"p\":";matrix(out,s.pos_end);
  out << ",\"v\":";matrix(out,s.vel_end);out << ",\"bg\":";matrix(out,s.bias_g);
  out << ",\"ba\":";matrix(out,s.bias_a);out << ",\"g\":";matrix(out,s.gravity);
  out << ",\"inv_exposure\":";number(out,s.inv_expo_time);
  out << ",\"cov19\":";matrix(out,s.cov);out << '}';
}
NativeDelta native_difference(const StatesGroup& a,const StatesGroup& b) {
  NativeDelta d;d.head<3>()=log_so3(b.rot_end.transpose()*a.rot_end);
  d.segment<3>(3)=a.pos_end-b.pos_end;d(6)=a.inv_expo_time-b.inv_expo_time;
  d.segment<3>(7)=a.vel_end-b.vel_end;d.segment<3>(10)=a.bias_g-b.bias_g;
  d.segment<3>(13)=a.bias_a-b.bias_a;d.segment<3>(16)=a.gravity-b.gravity;return d;
}
}
struct JointIntegration::Impl {
  std::string mode;
  std::ofstream log;
  JointFlow joint;
  Camera camera;
  RangeRay ray;
  double pixel_sigma=1.,range_sigma=.01,slope_sigma=.05,maximum_pair_s=.3;
  bool gyro_seeded=false,pair_pending=false;
  NativeCov gyro_native=NativeCov::Zero();
  int64_t pair_stamp=0,last_epoch=0;
  cv::Mat first_image;
  uint64_t track_sequence=0,epochs=0,pairs=0,flow_corrections=0,range_seeds=0;
  std::map<std::string,uint64_t> reasons;

  Impl(const rclcpp::Node::SharedPtr& node,const std::string& directory) {
    mode=node->declare_parameter<std::string>("research.flow_mode","native");
    if (mode!="native"&&mode!="joint_no_flow"&&mode!="full"&&mode!="zero_gyro"&&mode!="zero_plane")
      throw std::invalid_argument("Unknown research.flow_mode");
    pixel_sigma=node->declare_parameter<double>("research.pixel_sigma_px",1.);
    range_sigma=node->declare_parameter<double>("research.range_sigma_m",.01);
    slope_sigma=node->declare_parameter<double>("research.local_slope_sigma",.05);
    maximum_pair_s=node->declare_parameter<double>("research.maximum_pair_s",.3);
    if (!std::isfinite(pixel_sigma)||pixel_sigma<=0.||!std::isfinite(range_sigma)||range_sigma<=0.||
        !std::isfinite(slope_sigma)||slope_sigma<=0.||!std::isfinite(maximum_pair_s)||maximum_pair_s<=0.||maximum_pair_s>.3)
      throw std::invalid_argument("Invalid joint research noise/pair contract");
    camera.fx=camera.fy=277.128129211;camera.cx=159.5;camera.cy=119.5;
    camera.body_from_optical << 0.,-1.,0.,-1.,0.,0.,0.,0.,-1.;
    camera.optical_origin_body=V3(0.,0.,-.03);
    ray.origin_body=camera.optical_origin_body;ray.unit_direction_body=V3(0.,0.,-1.);
    const auto path=std::filesystem::path(directory)/"joint-updates.jsonl";
    if (std::filesystem::exists(path)) throw std::invalid_argument("Joint evidence file must be new");
    log.open(path);if (!log) throw std::runtime_error("Cannot open joint research evidence");
    log << "{\"event\":\"protocol\",\"schema_version\":1,\"mode\":";token(log,mode);
    log << ",\"state_order\":\"native19_clone6_plane3\",\"pixel_sigma_px\":";number(log,pixel_sigma);
    log << ",\"range_sigma_m\":";number(log,range_sigma);log << ",\"slope_prior_sigma\":";number(log,slope_sigma);
    log << ",\"maximum_pair_s\":";number(log,maximum_pair_s);
    log << ",\"camera_K\":[277.128129211,277.128129211,159.5,119.5],\"body_from_optical\":";matrix(log,camera.body_from_optical);
    log << ",\"optical_origin_body_m\":";matrix(log,camera.optical_origin_body);
    log << ",\"range_origin_body_m\":";matrix(log,ray.origin_body);
    log << ",\"range_direction_body\":";matrix(log,ray.unit_direction_body);
    log << ",\"noise_scope\":\"explicit_engineering_pixel_covariance_not_empirically_calibrated\""
        << ",\"range_scope\":\"exact_stamp_seed_only_not_all_50Hz_samples\""
        << ",\"pair_scope\":\"nonoverlapping_raw_two_view_pairs_exact_seed_then_next_camera_epoch\""
        << ",\"map_assumption\":\"native_fixed_map_and_independent_native_observations\""
        << ",\"world_plane_height_from_truth\":false,\"hardware_validated\":false}\n";
    flush();
  }
  void flush() { log.flush();if (!log) throw std::runtime_error("Joint evidence write failed"); }
  void simple(const std::string& event,int64_t stamp,const std::string& reason,bool accepted=false) {
    ++reasons[reason];log << "{\"event\":";token(log,event);log << ",\"stamp_ns\":" << stamp;
    log << ",\"accepted\":" << (accepted?"true":"false") << ",\"reason\":";token(log,reason);log << "}\n";flush();
  }
  void update_log(const std::string& event,int64_t stamp,const Update& u,const StatesGroup& before,
                  const StatesGroup& after,const JointCov& prior) {
    ++reasons[u.reason];log << "{\"event\":";token(log,event);log << ",\"stamp_ns\":" << stamp;
    log << ",\"seed_stamp_ns\":" << pair_stamp << ",\"accepted\":" << (u.accepted?"true":"false") << ",\"reason\":";token(log,u.reason);
    log << ",\"input_tracks\":" << u.input_tracks << ",\"accepted_tracks\":" << u.accepted_tracks;
    log << ",\"rejected_tracks\":" << u.rejected_tracks << ",\"nis\":";number(log,u.nis);
    log << ",\"dof\":" << u.dof << ",\"delta28\":";matrix(log,u.delta);
    log << ",\"native_delta_norm\":";number(log,native_difference(after,before).norm());
    log << ",\"native_covariance_delta\":";number(log,(after.cov-before.cov).norm());
    log << ",\"state_before\":";state_json(log,before);log << ",\"state_after\":";state_json(log,after);
    log << ",\"joint_covariance_before\":";matrix(log,prior);log << ",\"joint_covariance_after\":";matrix(log,joint.covariance());
    log << ",\"gyro_component_after\":";matrix(log,joint.gyro_covariance_component());
    log << ",\"plane_after\":";matrix(log,joint.plane());
    log << ",\"clone_R_after\":";matrix(log,joint.clone_rotation());log << ",\"clone_p_after\":";matrix(log,joint.clone_position());
    log << ",\"same_native_object\":true,\"native_state_address\":";
    std::ostringstream address;address << static_cast<const void*>(&after);token(log,address.str());
    if ((event=="flow"||event=="range_update")&&u.accepted) {
      const auto& d=joint.last_measurement();
      log << ",\"measurement_kind\":";token(log,d.kind);
      log << ",\"residual\":";matrix(log,d.residual);
      log << ",\"H\":";matrix(log,d.jacobian);log << ",\"R\":";matrix(log,d.noise);
      log << ",\"S\":";matrix(log,d.innovation);log << ",\"K\":";matrix(log,d.gain);
      log << ",\"accepted_track_ids\":[";
      for (size_t i=0;i<d.accepted_track_ids.size();++i) {if(i)log << ',';log << d.accepted_track_ids[i];}
      log << ']';
    }
    log << "}\n";flush();
  }
  void seed_gyro(const StatesGroup& state) {
    // Conditional covariance component generated by the initial gyro-bias
    // random variable.  This includes its existing cross terms and leaves a
    // PSD remainder, unlike selecting a correlated diagonal block in isolation.
    const Eigen::Matrix3d bg=state.cov.block<3,3>(10,10);
    Eigen::LDLT<Eigen::Matrix3d> factor(bg);
    if (factor.info()!=Eigen::Success||!factor.isPositive()) throw std::runtime_error("Initial gyro bias covariance is not invertible");
    gyro_native=state.cov.middleCols<3>(10)*factor.solve(state.cov.middleRows<3>(10));
    gyro_native=.5*(gyro_native+gyro_native.transpose()).eval();
    if (!covariance_valid(gyro_native)||!covariance_valid(state.cov-gyro_native))
      throw std::runtime_error("Invalid conditional initial gyro component");
    gyro_seeded=true;
  }
};

JointIntegration::JointIntegration(const rclcpp::Node::SharedPtr& node,const std::string& directory)
    :impl_(std::make_unique<Impl>(node,directory)) {}
JointIntegration::~JointIntegration() {
  if (!impl_) return;
  auto& p=*impl_;
  p.log << "{\"event\":\"summary\",\"epochs\":" << p.epochs << ",\"range_seed_augmentations\":" << p.range_seeds
        << ",\"completed_pairs\":" << p.pairs << ",\"nonzero_flow_state_corrections\":" << p.flow_corrections
        << ",\"pair_pending_at_shutdown\":" << (p.pair_pending?"true":"false") << ",\"reason_counts\":{";
  bool comma=false;for (const auto& item:p.reasons) { if (comma)p.log << ',';comma=true;token(p.log,item.first);p.log << ':' << item.second; }
  p.log << "}}\n";p.log.flush();
}
void JointIntegration::input(const std::string& sensor,int64_t stamp,bool accepted,const std::string& reason) {
  impl_->simple("input_"+sensor,stamp,reason,accepted);
}
void JointIntegration::propagated(StatesGroup& state,const Cov& f,const Cov& q,const Cov& qgyro,bool imu_ready,int64_t stamp) {
  auto& p=*impl_;if (p.mode=="native"||!imu_ready) return;
  if (!p.gyro_seeded) {
    if (q.norm()>1e-20||qgyro.norm()>1e-20||(f-NativeCov::Identity()).norm()>1e-20)
      throw std::runtime_error("First ready IMU epoch must be the unpropagated initialization return");
    p.seed_gyro(state);
    p.log << "{\"event\":\"initial_gyro_component\",\"stamp_ns\":" << stamp
          << ",\"definition\":\"conditional_initial_bg_component_before_first_imu_propagation\",\"cov19\":";matrix(p.log,state.cov);
    p.log << ",\"gyro_cov19\":";matrix(p.log,p.gyro_native);p.log << "}\n";p.flush();return;
  }
  if (p.joint.initialized()) {
    const Update u=p.joint.propagate_native(state,f,q,qgyro);
    if (!u.accepted) { p.simple("propagation_failure",p.last_epoch,u.reason);throw std::runtime_error(u.reason); }
    p.gyro_native=p.joint.gyro_covariance_component().topLeftCorner<19,19>();
  } else {
    p.gyro_native=(f*p.gyro_native*f.transpose()+qgyro).eval();
    p.gyro_native=.5*(p.gyro_native+p.gyro_native.transpose()).eval();
    if (!covariance_valid(p.gyro_native)||!covariance_valid(state.cov-p.gyro_native))
      throw std::runtime_error("Invalid pre-clone gyro component propagation");
  }
}
void JointIntegration::native_update(const StatesGroup& before,StatesGroup& after,const std::string& phase,int64_t stamp) {
  auto& p=*impl_;if (p.mode=="native"||!p.gyro_seeded)return;
  if (p.joint.initialized()) {
    const JointCov prior=p.joint.covariance();
    const Update u=p.joint.adopt_native_update(before,after);
    p.update_log("native_"+phase,stamp,u,before,after,prior);
    if (!u.accepted)throw std::runtime_error(u.reason);
    p.gyro_native=p.joint.gyro_covariance_component().topLeftCorner<19,19>();
  } else {
    if (!covariance_valid(after.cov,true)||!covariance_valid(before.cov-after.cov))
      throw std::runtime_error("Pre-clone native update is not a conditional covariance contraction");
    Eigen::LDLT<NativeCov> factor(before.cov);
    if (factor.info()!=Eigen::Success||!factor.isPositive())throw std::runtime_error("Pre-clone native prior not invertible");
    NativeCov transform=factor.solve(after.cov.transpose()).transpose();
    NativeCov reset=NativeCov::Identity();reset.topLeftCorner<3,3>()=right_reset(native_difference(after,before).head<3>());
    p.gyro_native=(reset*transform*p.gyro_native*transform.transpose()*reset.transpose()).eval();
    after.cov=(reset*after.cov*reset.transpose()).eval();after.cov=.5*(after.cov+after.cov.transpose()).eval();
    if (!covariance_valid(p.gyro_native)||!covariance_valid(after.cov-p.gyro_native))
      throw std::runtime_error("Invalid pre-clone conditional gyro component");
  }
}
void JointIntegration::epoch(int64_t stamp,StatesGroup& state,const cv::Mat& down,const EpochRange& range) {
  auto& p=*impl_;++p.epochs;
  if (stamp<=p.last_epoch)throw std::runtime_error("Non-increasing joint state epoch");
  p.last_epoch=stamp;
  if (range.available) {
    p.log << "{\"event\":\"epoch_range\",\"stamp_ns\":" << stamp << ",\"range_m\":";number(p.log,range.metres);
    p.log << ",\"variance_m2\":";number(p.log,p.range_sigma*p.range_sigma);
    p.log << ",\"frame\":";token(p.log,range.frame);p.log << ",\"exact_acquisition_stamp\":true}\n";p.flush();
  }
  if (p.mode=="native") { p.simple("epoch",stamp,"native_baseline_no_auxiliary_state");return; }
  if (down.empty()||down.type()!=CV_8UC1||down.cols!=320||down.rows!=240) {
    p.pair_pending=false;p.first_image.release();p.simple("epoch",stamp,"down_image_missing_or_invalid");return;
  }
  if (!p.pair_pending) {
    if (!range.available) { p.simple("epoch",stamp,"waiting_for_exact_stamp_seed_range");return; }
    const StatesGroup before(state);const JointCov prior=p.joint.covariance();
    const Update u=p.joint.initialize_from_range(state,p.ray,range.metres,p.range_sigma*p.range_sigma,
        Eigen::Vector2d::Zero(),Eigen::Matrix2d::Identity()*p.slope_sigma*p.slope_sigma,p.gyro_native);
    p.pair_stamp=stamp;p.update_log("range_seed",stamp,u,before,state,prior);
    if (u.accepted) { ++p.range_seeds;p.first_image=down.clone();p.pair_pending=true; }
    return;
  }
  const double dt=static_cast<double>(stamp-p.pair_stamp)*1e-9;
  p.pair_pending=false;++p.pairs;
  if (!(dt>0.&&dt<=p.maximum_pair_s)) { p.first_image.release();p.simple("flow",stamp,"pair_interval_outside_contract");return; }
  std::vector<cv::Point2f> first,second,back;
  cv::goodFeaturesToTrack(p.first_image,first,80,.01,12.);
  std::vector<unsigned char> ok,back_ok;std::vector<float> errors,back_errors;
  if (!first.empty()) {
    cv::calcOpticalFlowPyrLK(p.first_image,down,first,second,ok,errors,cv::Size(21,21),3,
        cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,30,.01));
    cv::calcOpticalFlowPyrLK(down,p.first_image,second,back,back_ok,back_errors,cv::Size(21,21),3,
        cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,30,.01));
  }
  std::vector<Track> tracks;
  for (size_t i=0;i<first.size();++i) {
    if (!ok[i]||!back_ok[i]||!std::isfinite(errors[i])||errors[i]>20.||cv::norm(first[i]-back[i])>1.)continue;
    if (first[i].x<8||first[i].x>311||first[i].y<8||first[i].y>231||
        second[i].x<8||second[i].x>311||second[i].y<8||second[i].y>231)continue;
    Track t;t.id=++p.track_sequence;t.first_pixel=Eigen::Vector2d(first[i].x,first[i].y);
    t.second_pixel=Eigen::Vector2d(second[i].x,second[i].y);
    t.first_cov=t.second_cov=Eigen::Matrix2d::Identity()*p.pixel_sigma*p.pixel_sigma;tracks.push_back(t);
  }
  p.first_image.release();
  p.log << "{\"event\":\"raw_tracks\",\"first_stamp_ns\":" << p.pair_stamp << ",\"second_stamp_ns\":" << stamp
        << ",\"detected\":" << first.size() << ",\"gyro_precompensated\":false,\"tracks\":[";
  for(size_t i=0;i<tracks.size();++i) {
    if(i)p.log << ',';
    p.log << "{\"id\":" << tracks[i].id << ",\"u0\":";matrix(p.log,tracks[i].first_pixel);
    p.log << ",\"u1\":";matrix(p.log,tracks[i].second_pixel);p.log << '}';
  }
  p.log << "]}\n";p.flush();
  if(p.mode=="joint_no_flow") { p.simple("flow",stamp,"joint_no_flow_control");return; }
  const JointCov original=p.joint.covariance();
  const CrossAblation ablation=p.mode=="zero_gyro"?CrossAblation::ZeroGyroCross:
      (p.mode=="zero_plane"?CrossAblation::ZeroPlaneCross:CrossAblation::None);
  if(!p.joint.apply_cross_ablation(ablation)) { p.simple("flow",stamp,"cross_ablation_would_make_covariance_indefinite");return; }
  if (ablation!=CrossAblation::None) {
    p.log << "{\"event\":\"cross_ablation\",\"stamp_ns\":" << stamp << ",\"mode\":";token(p.log,p.mode);
    p.log << ",\"covariance_before\":";matrix(p.log,original);p.log << ",\"covariance_after\":";matrix(p.log,p.joint.covariance());p.log << "}\n";p.flush();
  }
  const StatesGroup before(state);const JointCov prior=p.joint.covariance();
  p.log << "{\"event\":\"flow_prior\",\"stamp_ns\":" << stamp << ",\"seed_stamp_ns\":" << p.pair_stamp;
  p.log << ",\"state\":";state_json(p.log,state);p.log << ",\"P\":";matrix(p.log,prior);
  p.log << ",\"Pg\":";matrix(p.log,p.joint.gyro_covariance_component());
  p.log << ",\"clone_R\":";matrix(p.log,p.joint.clone_rotation());p.log << ",\"clone_p\":";matrix(p.log,p.joint.clone_position());
  p.log << ",\"plane\":";matrix(p.log,p.joint.plane());p.log << ",\"plane_anchor\":";matrix(p.log,p.joint.plane_anchor_world());p.log << "}\n";p.flush();
  const Update u=p.joint.update_tracks(state,p.camera,tracks);
  p.update_log("flow",stamp,u,before,state,prior);
  if(u.accepted&&native_difference(state,before).norm()>1e-12&&(state.cov-before.cov).norm()>1e-12)++p.flow_corrections;
  p.gyro_native=p.joint.gyro_covariance_component().topLeftCorner<19,19>();
}
} // namespace impact_research
