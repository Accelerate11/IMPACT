#include "research_serial_position.h"
#include <iostream>
#include <random>
using namespace impact_research;
struct Native {
  M3 rot_end=M3::Identity();V3 pos_end=V3(0.,0.,1.7),vel_end=V3(.5,.1,0.);
  V3 bias_g=V3::Zero(),bias_a=V3::Zero(),gravity=V3(0.,0.,-9.81);
  double inv_expo_time=1.;NativeCov cov=NativeCov::Identity()*.01;
};
int checks=0;
void check(bool yes,const std::string& text){++checks;if(!yes)throw std::runtime_error(text);}
Camera camera(){Camera c;c.fx=230.;c.fy=225.;c.cx=160.;c.cy=120.;
 c.body_from_optical<<0.,-1.,0.,-1.,0.,0.,0.,0.,-1.;c.optical_origin_body=V3(.02,-.01,-.03);return c;}
JointFlow initialize(Native& n,double scale=1.){
 n.cov*=scale;JointFlow f;RangeRay ray;ray.origin_body=camera().optical_origin_body;
 auto r=f.initialize_from_range(n,ray,1.67,.0001*scale,Eigen::Vector2d(.04,-.03),
     Eigen::Matrix2d::Identity()*.0025*scale);
 check(r.accepted,r.reason);return f;
}
void move(JointFlow& f,Native& n,const V3& error=V3::Zero(),const V3& rotation_error=V3::Zero()){
 NativeCov trans=NativeCov::Identity();trans.block<3,3>(3,7)=M3::Identity()*.08;
 NativeCov q=NativeCov::Identity()*1e-6;n.cov=trans*n.cov*trans.transpose()+q;
 n.pos_end+=V3(.06,-.025,.008)+error;n.rot_end=n.rot_end*exp_so3(V3(.015,-.012,.018))*exp_so3(rotation_error);
 auto r=f.propagate_native(n,trans,q);check(r.accepted,r.reason);
}
std::vector<Track> observations(const JointFlow& truth){
 std::vector<Track> out;
 for(int y=0;y<4;++y)for(int x=0;x<4;++x){Track t;t.id=out.size();
  t.first_pixel=Eigen::Vector2d(105.+32.*x,75.+28.*y);auto p=truth.project(camera(),t.first_pixel);
  check(p.valid,p.reason);t.second_pixel=p.pixel;t.first_cov=Eigen::Matrix2d::Identity()*.2;
  t.second_cov=Eigen::Matrix2d::Identity()*.3;out.push_back(t);}
 return out;
}
void recover_and_update(){
 Native truth,estimated;JointFlow gt=initialize(truth),f=initialize(estimated);
 move(gt,truth);move(f,estimated,V3(.024,-.012,.008));const auto tracks=observations(gt);
 const auto prior=f.covariance();const Native before=estimated;
 const auto fit=fit_serial_position(f,estimated,camera(),tracks);
 check(fit.accepted,fit.reason);check((fit.position-truth.pos_end).norm()<2e-6,"position fit recovers perturbed nominal from raw pixels");
 check(fit.iterations<=12&&fit.line_search_evaluations<=96,"declared finite iteration budget");
 check(fit.accepted_track_ids.size()==16,"identities of all fitted observations recorded");
 check(fit.nuisance_covariance_part.norm()>fit.pixel_covariance_part.norm(),"uncertain attitude clone and plane retained");
 check((fit.covariance-fit.pixel_covariance_part-fit.nuisance_covariance_part).norm()<1e-12,"position covariance separates pixel and full nuisance contributions");
 check((fit.influence*fit.position_jacobian-M3::Identity()).norm()<1e-9,"influence is identifiable position estimator");
 const auto applied=apply_serial_position(f,estimated,fit);
 check(applied.update.accepted,applied.update.reason);
 check((estimated.pos_end-before.pos_end).norm()>1e-6,"serial pipeline mutates actual native nominal");
 check((estimated.cov-before.cov).norm()>1e-6,"serial pipeline mutates actual native covariance");
 check((estimated.pos_end-truth.pos_end).norm()<(before.pos_end-truth.pos_end).norm(),"serial correction improves known deterministic perturbation");
 check((applied.native_gain*applied.position_innovation-applied.update.delta.head<kNative>()).norm()<1e-12,"native measurement K residual reconstructs injection");
 check(covariance_valid(f.covariance()),"serial native update and reset preserve joint PSD");
 Native full_native;JointFlow full=initialize(full_native);
 // A second independently constructed full-filter prior has exactly the same
 // nominal/covariance inputs, then applies raw pixels directly.
 move(full,full_native,V3(.024,-.012,.008));
 check((full.covariance()-prior).norm()<1e-12,"full and serial start from same joint prior");
 const auto raw=full.update_tracks(full_native,camera(),tracks);
 check(raw.accepted,raw.reason);
 check((full_native.cov-estimated.cov).norm()>1e-7,"serial position stage is different from relabeled full raw-pixel EKF");
}
void nuisance_and_outliers(){
 Native truth,a,b;JointFlow gt=initialize(truth),fa=initialize(a),fb=initialize(b,4.);
 move(gt,truth);move(fa,a,V3(.02,-.01,.006));move(fb,b,V3(.02,-.01,.006));
 auto tracks=observations(gt);const auto fit_a=fit_serial_position(fa,a,camera(),tracks),fit_b=fit_serial_position(fb,b,camera(),tracks);
 check(fit_a.accepted,fit_a.reason);check(fit_b.accepted,fit_b.reason);
 check((fit_a.position-fit_b.position).norm()<1e-10,"nominal position fit does not use uncertain priors as geometric truth");
 check((fit_b.covariance-fit_a.covariance).trace()>0.,"larger pose range and plane uncertainty increases measurement uncertainty");
 const Eigen::MatrixXd diagonal=fit_a.nuisance_covariance.diagonal().asDiagonal();
 const Eigen::MatrixXd bmap=-fit_a.influence*fit_a.nuisance_jacobian;
 check((fit_a.nuisance_covariance_part-bmap*diagonal*bmap.transpose()).norm()>1e-7,"all nuisance mutual cross covariance is retained");
 tracks[0].second_pixel+=Eigen::Vector2d(45.,-35.);
 const auto robust=fit_serial_position(fa,a,camera(),tracks);
 check(robust.accepted,robust.reason);
 check(robust.accepted_track_ids.size()==15,"declared robust trim rejects isolated corrupted track");
 check((robust.position-truth.pos_end).norm()<2e-6,"robust fit recovers position with corrupted correspondence");
 Native uncertain;JointFlow fu=initialize(uncertain);move(fu,uncertain,V3(.02,-.01,.006),V3(0.,0.,.04));
 const auto uncertain_fit=fit_serial_position(fu,uncertain,camera(),observations(gt));
 check(uncertain_fit.accepted,uncertain_fit.reason);
 check(uncertain_fit.nuisance_covariance_part.norm()>0.,"perturbed attitude is an uncertain fixed mean, never truth");
}
void covariance_monte_carlo(){
 Native truth,n;JointFlow gt=initialize(truth),f=initialize(n);move(gt,truth);move(f,n,V3(.01,-.005,.003));
 const auto fit=fit_serial_position(f,n,camera(),observations(gt));check(fit.accepted,fit.reason);
 Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> nuisance(fit.nuisance_covariance);
 check(nuisance.info()==Eigen::Success,"nuisance covariance factorization");
 const Eigen::MatrixXd root=nuisance.eigenvectors()*nuisance.eigenvalues().cwiseMax(0.).cwiseSqrt().asDiagonal();
 const Eigen::MatrixXd rroot=fit.pixel_covariance.llt().matrixL();
 std::mt19937 generator(7719);std::normal_distribution<double> normal(0.,1.);
 const int samples=30000;V3 mean=V3::Zero();M3 moment=M3::Zero();
 for(int i=0;i<samples;++i){Eigen::VectorXd raw(fit.pixel_covariance.rows()),eta(12);
  for(int j=0;j<raw.size();++j)raw(j)=normal(generator);
  for(int j=0;j<12;++j)eta(j)=normal(generator);
  const V3 error=fit.influence*(rroot*raw-fit.nuisance_jacobian*root*eta);
  mean+=error;moment+=error*error.transpose();}
 const M3 empirical=moment/samples-(mean/samples)*(mean/samples).transpose();
 check((empirical-fit.covariance).norm()/fit.covariance.norm()<.035,"30000 seeded conditional linearized covariance samples agree");
}
void rejection(){
 Native n;JointFlow f=initialize(n);auto tracks=observations(f);
 auto pure=fit_serial_position(f,n,camera(),tracks);check(!pure.accepted&&pure.reason=="pure_rotation_or_insufficient_translation","pure rotation rejects translation fit");
 move(f,n);tracks=observations(f);for(auto& t:tracks)t.first_cov.setZero();
 auto unknown=fit_serial_position(f,n,camera(),tracks);check(!unknown.accepted,"unknown pixel covariance is rejected");
 tracks=observations(f);tracks[1].id=tracks[0].id;
 check(!fit_serial_position(f,n,camera(),tracks).accepted,"duplicate identity rejected");
 SerialFit fake;fake.accepted=true;fake.position=n.pos_end+V3(10.,0.,0.);fake.covariance=M3::Identity()*1e-5;
 const Native before=n;const JointCov p=f.covariance();const auto bad=apply_serial_position(f,n,fake);
 check(!bad.update.accepted&&bad.update.reason=="serial_native_position_NIS_gate","native preupdate NIS rejects inconsistent serial solution");
 check((n.pos_end-before.pos_end).norm()==0.&&(n.cov-before.cov).norm()==0.&&(f.covariance()-p).norm()==0.,"rejected serial fusion changes no state or covariance");
}
int main(){try{recover_and_update();nuisance_and_outliers();covariance_monte_carlo();rejection();
 std::cout<<"PASS "<<checks<<" checks; serial robust position fit, uncertain nuisance covariance, actual native update and rejection\n";return 0;
 }catch(const std::exception& e){std::cerr<<"FAIL after "<<checks<<" checks: "<<e.what()<<'\n';return 1;}}
