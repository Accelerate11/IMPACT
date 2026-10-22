#include "research_joint_flow.h"
#include <iostream>
#include <random>

using namespace impact_research;
struct Native {
  M3 rot_end=M3::Identity();
  V3 pos_end=V3(0.,0.,1.7),vel_end=V3(.5,.1,0.);
  V3 bias_g=V3::Zero(),bias_a=V3::Zero(),gravity=V3(0.,0.,-9.81);
  double inv_expo_time=1.;
  NativeCov cov=NativeCov::Identity()*.01;
};
int checks=0;
void check(bool condition,const std::string& message) {
  ++checks; if(!condition) throw std::runtime_error(message);
}
Camera camera() {
  Camera c; c.fx=230.; c.fy=225.; c.cx=160.; c.cy=120.;
  c.body_from_optical << 0.,-1.,0.,-1.,0.,0.,0.,0.,-1.;
  c.optical_origin_body=V3(.02,-.01,-.03); return c;
}
RangeRay range_ray() { RangeRay r; r.origin_body=camera().optical_origin_body; return r; }
JointFlow seeded(Native& s) {
  JointFlow f;
  NativeCov gyro=NativeCov::Zero(); gyro.block<3,3>(0,0)=s.cov.block<3,3>(0,0)*.2;
  gyro.block<3,3>(10,10)=s.cov.block<3,3>(10,10)*.3;
  const auto result=f.initialize_from_range(s,range_ray(),1.67,.0001,
      Eigen::Vector2d(.04,-.03),Eigen::Matrix2d::Identity()*.0025,gyro);
  check(result.accepted,result.reason); return f;
}
void advance(JointFlow& f,Native& s) {
  NativeCov transition=NativeCov::Identity();
  transition.block<3,3>(3,7)=M3::Identity()*.08;
  transition.block<3,3>(0,10)=-M3::Identity()*.08;
  NativeCov q=NativeCov::Identity()*1e-6,qg=NativeCov::Zero();
  qg.block<3,3>(0,0)=M3::Identity()*2e-7;
  s.cov=transition*s.cov*transition.transpose()+q;
  s.pos_end+=V3(.06,-.025,.008);
  s.rot_end=s.rot_end*exp_so3(V3(.015,-.012,.018));
  const auto result=f.propagate_native(s,transition,q,qg);
  check(result.accepted,result.reason);
}
void test_jacobians() {
  Native s; s.rot_end=exp_so3(V3(.03,-.025,.12));
  JointFlow f=seeded(s); advance(f,s); const auto c=camera();
  for(const Eigen::Vector2d& pixel : {Eigen::Vector2d(110.,80.),Eigen::Vector2d(195.,152.),Eigen::Vector2d(158.,118.)}) {
    const auto expected=f.project(c,pixel); check(expected.valid,expected.reason);
    for(int column=0;column<kJoint;++column) {
      const double epsilon=1e-6; JointDelta d=JointDelta::Zero(); d(column)=epsilon;
      JointFlow plus=f,minus=f; plus.diagnostic_perturb(d); minus.diagnostic_perturb(-d);
      const auto p=plus.project(c,pixel),m=minus.project(c,pixel);
      check(p.valid&&m.valid,"finite-difference domain");
      const Eigen::Vector2d numerical=(p.pixel-m.pixel)/(2.*epsilon);
      check((numerical-expected.jacobian.col(column)).norm()<2e-5,"28D projection Jacobian "+std::to_string(column));
    }
    for(int column=0;column<2;++column) {
      const double epsilon=1e-4; Eigen::Vector2d d=Eigen::Vector2d::Zero(); d(column)=epsilon;
      const auto p=f.project(c,pixel+d),m=f.project(c,pixel-d);
      check(((p.pixel-m.pixel)/(2.*epsilon)-expected.first_pixel_jacobian.col(column)).norm()<1e-7,"initial endpoint covariance Jacobian");
    }
  }
}
void test_range_augmentation_monte_carlo() {
  Native s; s.rot_end=exp_so3(V3(.06,-.03,.04)); s.cov*=.0001;
  JointFlow f=seeded(s);
  const auto ray=range_ray(); const V3 slopes(.04,-.03,0.);
  const V3 q=s.pos_end+s.rot_end*(ray.origin_body+1.67*ray.unit_direction_body);
  const double height=q.z()-slopes.x()*q.x()-slopes.y()*q.y();
  check(std::abs(f.plane().z()-height)<1e-12,"range sets plane height without truth");
  check((f.covariance().block<6,6>(kClone,kClone)-s.cov.topLeftCorner<6,6>()).norm()<1e-14,"clone preserves native covariance");
  check(f.covariance().block<kNative,3>(0,kPlane).norm()>0.,"range/plane native cross retained");
  check(covariance_valid(f.covariance()),"singular clone augmentation PSD");
  std::mt19937 generator(1241); std::normal_distribution<double> normal(0.,1.);
  const int samples=45000; Eigen::Matrix<double,22,1> mean=Eigen::Matrix<double,22,1>::Zero();
  Eigen::Matrix<double,22,22> moment=Eigen::Matrix<double,22,22>::Zero();
  for(int sample=0;sample<samples;++sample) {
    NativeDelta dx; for(int j=0;j<kNative;++j) dx(j)=std::sqrt(s.cov(j,j))*normal(generator);
    const double da=.05*normal(generator),db=.05*normal(generator),dr=.01*normal(generator);
    const V3 qp=s.pos_end+dx.segment<3>(3)+s.rot_end*exp_so3(dx.head<3>())*
        (ray.origin_body+(1.67+dr)*ray.unit_direction_body);
    Eigen::Matrix<double,22,1> v; v.head<kNative>()=dx; v(19)=da;v(20)=db;
    v(21)=qp.z()-(slopes.x()+da)*qp.x()-(slopes.y()+db)*qp.y()-height;
    mean+=v;moment+=v*v.transpose();
  }
  const Eigen::Matrix<double,22,22> empirical=moment/samples-(mean/samples)*(mean/samples).transpose();
  Eigen::Matrix<double,22,22> analytical;
  analytical.topLeftCorner<kNative,kNative>()=f.covariance().topLeftCorner<kNative,kNative>();
  analytical.topRightCorner<kNative,3>()=f.covariance().block<kNative,3>(0,kPlane);
  analytical.bottomLeftCorner<3,kNative>()=f.covariance().block<3,kNative>(kPlane,0);
  analytical.bottomRightCorner<3,3>()=f.covariance().bottomRightCorner<3,3>();
  check((empirical-analytical).norm()/analytical.norm()<.035,"joint augmentation empirical covariance agreement");
}
std::vector<Track> tracks_for(const JointFlow& f,double shift=0.) {
  std::vector<Track> tracks;
  for(int y=0;y<3;++y) for(int x=0;x<4;++x) {
    Track t; t.id=tracks.size()+1;t.first_pixel=Eigen::Vector2d(110.+30.*x,80.+35.*y);
    const auto p=f.project(camera(),t.first_pixel); check(p.valid,p.reason);
    t.second_pixel=p.pixel+Eigen::Vector2d(shift,-.4*shift);
    t.first_cov=Eigen::Matrix2d::Identity()*.15;t.second_cov=Eigen::Matrix2d::Identity()*.2;
    tracks.push_back(t);
  }
  return tracks;
}
void test_actual_state_update_and_degeneracy() {
  Native s; JointFlow f=seeded(s); advance(f,s); const Native before=s;
  auto tracks=tracks_for(f,.5);
  const auto update=f.update_tracks(s,camera(),tracks);
  check(update.accepted,update.reason);check(update.native_delta_norm>1e-8,"flow changes actual native state");
  check((s.pos_end-before.pos_end).norm()>1e-8,"native pose mutated");
  check((s.cov-before.cov).norm()>1e-8,"native covariance mutated");
  check((s.cov-f.covariance().topLeftCorner<kNative,kNative>()).norm()<1e-14,"native full joint P identical");
  check(covariance_valid(f.covariance()),"Joseph plus manifold reset PSD");
  check(f.accepted_flow_updates()==1,"actual accepted update counter");
  check(f.last_measurement().applied&&f.last_measurement().accepted_track_ids.size()==tracks.size(),"accepted IDs bound to actual update");
  check((f.last_measurement().gain*f.last_measurement().residual-update.delta).norm()<1e-12,"recorded K residual reconstructs applied delta");
  check(f.last_measurement().jacobian.cols()==kJoint&&f.last_measurement().noise.rows()==24,"accepted batch full H R recorded");
  Native pure; JointFlow pure_filter=seeded(pure);
  auto pure_tracks=tracks_for(pure_filter); const Native unchanged=pure;
  const auto reject=pure_filter.update_tracks(pure,camera(),pure_tracks);
  check(!reject.accepted&&reject.reason=="pure_rotation_or_insufficient_translation","pure rotation rejected before scale update");
  check((pure.cov-unchanged.cov).norm()==0.,"degenerate rejection does not shrink covariance");
  tracks=tracks_for(f);for(auto& t:tracks) t.first_cov.setZero();
  const Native current=s; const auto bad=f.update_tracks(s,camera(),tracks);
  check(!bad.accepted,"unknown endpoint covariance rejected");check((s.cov-current.cov).norm()==0.,"unknown covariance rejection immutable");
  check(!f.last_measurement().applied&&f.last_measurement().gain.size()==0,"rejection does not expose stale accepted matrices");
  tracks=tracks_for(f,1000.);const auto outlier=f.update_tracks(s,camera(),tracks);
  check(!outlier.accepted&&outlier.rejected_tracks==tracks.size(),"preupdate NIS rejects outliers");
  check((s.cov-current.cov).norm()==0.,"outlier rejection immutable");
}
void test_native_conditional_and_tags() {
  Native s; JointFlow f=seeded(s);advance(f,s); const Native before=s;
  Eigen::Matrix<double,2,kNative> h=Eigen::Matrix<double,2,kNative>::Zero();h(0,3)=1.;h(1,0)=1.;
  const Eigen::Matrix2d noise=Eigen::Matrix2d::Identity()*.03;
  const Eigen::Matrix<double,kNative,2> gain=before.cov*h.transpose()*(h*before.cov*h.transpose()+noise).inverse();
  Eigen::Vector2d residual(.02,.01); const NativeDelta dx=gain*residual;
  s.rot_end=s.rot_end*exp_so3(dx.head<3>());s.pos_end+=dx.segment<3>(3);
  s.vel_end+=dx.segment<3>(7);s.bias_g+=dx.segment<3>(10);
  s.cov=(NativeCov::Identity()-gain*h)*before.cov;
  const JointCov prior=f.covariance(),gyro=f.gyro_covariance_component();
  Eigen::Matrix<double,2,kJoint> full_h=Eigen::Matrix<double,2,kJoint>::Zero();
  full_h.leftCols<kNative>()=h;
  const Eigen::Matrix<double,kJoint,2> full_gain=prior*full_h.transpose()*
      (full_h*prior*full_h.transpose()+noise).inverse();
  const JointDelta expected_delta=full_gain*residual;
  JointCov reset=JointCov::Identity();reset.topLeftCorner<3,3>()=right_reset(expected_delta.head<3>());
  reset.block<3,3>(kClone,kClone)=right_reset(expected_delta.segment<3>(kClone));
  const JointCov full_l=JointCov::Identity()-full_gain*full_h;
  const JointCov expected_cov=reset*(full_l*prior*full_l.transpose()+full_gain*noise*full_gain.transpose())*reset.transpose();
  const auto adopted=f.adopt_native_update(before,s);
  check(adopted.accepted,adopted.reason);
  check(adopted.delta.tail<9>().norm()>0.,"native measurement conditions clone/plane nominal");
  check((prior-f.covariance()).norm()>1e-8,"native measurement conditions full covariance");
  check((gyro-f.gyro_covariance_component()).norm()>1e-9,"tag uses full native update transformation");
  check(covariance_valid(f.covariance()),"native conditional plus reset PSD");
  check((adopted.delta-expected_delta).norm()<1e-12,"native conditional delta equals direct full joint EKF");
  check((f.covariance()-expected_cov).norm()<1e-12,"native conditional covariance equals direct Joseph full joint EKF");
  check((f.gyro_covariance_component()-reset*full_l*gyro*full_l.transpose()*reset.transpose()).norm()<1e-12,"native tag equals full joint measurement transform");
  JointFlow gyro_zero=f,plane_zero=f;
  check(gyro_zero.apply_cross_ablation(CrossAblation::ZeroGyroCross),"gyro component ablation PSD");
  check(plane_zero.apply_cross_ablation(CrossAblation::ZeroPlaneCross),"plane cross ablation PSD");
  check((gyro_zero.covariance().topLeftCorner<kNative,kNative>()-f.covariance().topLeftCorner<kNative,kNative>()).norm()==0.,"ablation preserves native marginal");
  check(plane_zero.covariance().topRightCorner<kPlane,3>().norm()==0.,"scale ablation plane cross only");
  check(gyro_zero.covariance().block<kNative,6>(0,kClone).norm()>0.,"gyro ablation retains non-gyro clone correlation");
  check(gyro_zero.gyro_covariance_component().norm()>0.,"gyro ablation preserves tagged marginal for later pairs");
  check(covariance_valid(gyro_zero.covariance()-gyro_zero.gyro_covariance_component()),"gyro ablation preserves PSD remainder");
  check(covariance_valid(plane_zero.covariance()-plane_zero.gyro_covariance_component()),"plane ablation preserves PSD remainder");
  const NativeCov inherited_tag=gyro_zero.gyro_covariance_component().topLeftCorner<kNative,kNative>();
  Native next_pair_native=s;
  JointFlow next_pair;
  const auto pair_init=next_pair.initialize_from_range(next_pair_native,range_ray(),1.675,.0001,
      Eigen::Vector2d::Zero(),Eigen::Matrix2d::Identity()*.0025,inherited_tag);
  check(pair_init.accepted,pair_init.reason);
  check((next_pair.gyro_covariance_component().topLeftCorner<kNative,kNative>()-inherited_tag).norm()<1e-14,"new pair inherits original bg tag component");
  const auto next=f.update_range(s,range_ray(),1.675,.0001);
  check(next.accepted,next.reason);check(f.accepted_range_updates()==1,"new range applied once by caller");
  const Native saved=s; const auto bad=f.update_range(s,range_ray(),1.675,0.);
  check(!bad.accepted,"unknown range variance rejected"); check((s.cov-saved.cov).norm()==0.,"range rejection immutable");
  Native mismatch=s;mismatch.cov(0,0)+=.01;
  const auto propagation=f.propagate_native(mismatch,NativeCov::Identity(),NativeCov::Zero());
  check(!propagation.accepted,"unreported native covariance mutation rejected");
}
double independent_range(const Native& state,const V3& plane) {
  const auto ray=range_ray();const V3 n(-plane.x(),-plane.y(),1.);
  const V3 o=state.pos_end+state.rot_end*ray.origin_body;
  const V3 d=state.rot_end*ray.unit_direction_body;
  return (plane.z()-n.dot(o))/n.dot(d);
}
void test_range_jacobian() {
  Native s;s.rot_end=exp_so3(V3(.06,-.05,.02));JointFlow f=seeded(s);advance(f,s);
  const Native before=s;const V3 plane=f.plane();
  const double measured=independent_range(s,plane)+.002;
  const auto update=f.update_range(s,range_ray(),measured,.0001);
  check(update.accepted,update.reason);const auto jacobian=f.last_measurement().jacobian;
  check(f.last_measurement().kind=="range"&&f.last_measurement().applied,"range matrix diagnostics bound");
  for(int column=0;column<kJoint;++column) {
    const double epsilon=1e-6;Native plus=before,minus=before;V3 pp=plane,pm=plane;
    if(column<3) { V3 d=V3::Zero();d(column)=epsilon;plus.rot_end=before.rot_end*exp_so3(d);minus.rot_end=before.rot_end*exp_so3(-d); }
    else if(column>=3&&column<6) { plus.pos_end(column-3)+=epsilon;minus.pos_end(column-3)-=epsilon; }
    else if(column>=kPlane) { pp(column-kPlane)+=epsilon;pm(column-kPlane)-=epsilon; }
    const double numerical=(independent_range(plus,pp)-independent_range(minus,pm))/(2.*epsilon);
    check(std::abs(numerical-jacobian(0,column))<2e-8,"range ray plane analytic Jacobian "+std::to_string(column));
  }
}
void test_so3_reset() {
  const V3 d(.13,-.07,.05);
  for(int column=0;column<3;++column) {
    V3 e=V3::Zero();e(column)=1e-6;
    const V3 plus=log_so3(exp_so3(-d)*exp_so3(d+e));
    const V3 minus=log_so3(exp_so3(-d)*exp_so3(d-e));
    check(((plus-minus)/(2e-6)-right_reset(d).col(column)).norm()<1e-8,"right error manifold reset derivative");
  }
}
int main() {
  try {
    test_jacobians();test_range_augmentation_monte_carlo();
    test_actual_state_update_and_degeneracy();test_native_conditional_and_tags();test_range_jacobian();test_so3_reset();
    std::cout << "PASS " << checks << " checks; actual nonlinear joint state updates, Jacobians, covariance, rejection and tags\n";
    return 0;
  } catch(const std::exception& error) {
    std::cerr << "FAIL after " << checks << " checks: " << error.what() << '\n';return 1;
  }
}
