// Actual upstream ImuProcess behavior fixture, linked against xq_livo_core.
// Proves propagation with no new cloud; does not substitute for native replay.
#include "IMU_Processing.h"
#include <iostream>
#include <stdexcept>

int checks=0;
void check(bool condition,const char* description) {
  ++checks;if(!condition)throw std::runtime_error(description);
}
sensor_msgs::msg::Imu::SharedPtr imu(double seconds) {
  auto sample=std::make_shared<sensor_msgs::msg::Imu>();
  const int64_t ns=static_cast<int64_t>(std::llround(seconds*1e9));
  sample->header.stamp.sec=static_cast<int32_t>(ns/1000000000);
  sample->header.stamp.nanosec=static_cast<uint32_t>(ns%1000000000);
  sample->linear_acceleration.z=9.81;
  sample->angular_velocity.z=.3;
  return sample;
}
void initialize(ImuProcess& process,StatesGroup& state,LidarMeasureGroup& group,PointCloudXYZI::Ptr& output) {
  process.set_imu_init_frame_num(1);
  process.set_extrinsic(V3D::Zero(),M3D::Identity());
  group.lio_vio_flg=LIO;MeasureGroup measurement;measurement.lio_time=100.01;
  for(double time:{100.,100.005,100.01})measurement.imu.push_back(imu(time));
  group.measures.clear();group.measures.push_back(measurement);
  output=std::make_shared<PointCloudXYZI>();process.Process2(group,state,output);
  check(!process.imu_need_init,"IMU initialized by actual Process2");
  check(process.research_propagation_steps==0,"initialization return has no hidden propagation");
  state.pos_end=V3D(1.,2.,1.5);state.vel_end=V3D(.4,-.1,0.);
}
void propagate(ImuProcess& process,StatesGroup& state,LidarMeasureGroup& group,
               const PointCloudXYZI::Ptr& output,double begin,double end,bool cloud) {
  group.lio_vio_flg=LIO;group.pcl_proc_cur->clear();
  if(cloud) {
    for(float fraction:{.25f,.5f,.75f,1.f}) {
      PointType p;p.x=.4f;p.y=.3f;p.z=2.f;
      p.curvature=static_cast<float>((end-begin)*1000.)*fraction;
      group.pcl_proc_cur->push_back(p);
    }
  }
  MeasureGroup measurement;measurement.lio_time=end;
  const int steps=static_cast<int>(std::llround((end-begin)/.005));
  for(int i=1;i<=steps;++i)measurement.imu.push_back(imu(begin+.005*i));
  group.measures.clear();group.measures.push_back(measurement);
  process.Process2(group,state,output);
}
int main() {
  try {
    ImuProcess no_cloud,control;StatesGroup actual,reference;LidarMeasureGroup a,b;
    PointCloudXYZI::Ptr out_a,out_b;
    initialize(no_cloud,actual,a,out_a);initialize(control,reference,b,out_b);
    const StatesGroup before=actual;
    propagate(no_cloud,actual,a,out_a,100.01,100.03,false);
    propagate(control,reference,b,out_b,100.01,100.03,true);
    check(out_a->empty(),"empty LIO never duplicates prior cloud");
    check(no_cloud.research_propagation_steps==4,"empty LIO processes every real IMU interval");
    check(std::abs(a.last_lio_update_time-100.03)<1e-10,"empty LIO advances native epoch");
    check((actual.pos_end-V3D(1.008,1.998,1.5)).norm()<1e-9,"empty LIO advances physical constant-velocity position");
    check(std::abs(Eigen::AngleAxisd(actual.rot_end).angle()-.006)<1e-10,"empty LIO integrates gyro rotation");
    const auto predicted=no_cloud.research_transition*before.cov*no_cloud.research_transition.transpose()+no_cloud.research_process_noise;
    check((predicted-actual.cov).norm()<1e-12,"actual native F Q reconstruct empty-LIO covariance");
    check(no_cloud.research_gyro_process_noise.norm()>0.,"empty LIO exports actual gyro process component");
    check((actual.pos_end-reference.pos_end).norm()<1e-12,"cloud presence does not change IMU nominal");
    check((actual.cov-reference.cov).norm()<1e-12,"cloud presence does not change IMU covariance");
    // Native LIVO schedules an immediate VIO state at the same acquisition time.
    a.lio_vio_flg=VIO;MeasureGroup image;image.vio_time=100.03;
    a.measures.clear();a.measures.push_back(image);const StatesGroup before_vio=actual;
    no_cloud.Process2(a,actual,out_a);
    check(no_cloud.research_propagation_steps==0,"same-stamp VIO does not repeat IMU integration");
    check((actual.cov-before_vio.cov).norm()==0.,"same-stamp VIO has no duplicate process covariance");
    check((actual.pos_end-before_vio.pos_end).norm()==0.,"same-stamp VIO nominal remains at LIO epoch");
    propagate(no_cloud,actual,a,out_a,100.03,100.05,true);
    propagate(control,reference,b,out_b,100.03,100.05,true);
    check(out_a->size()==4&&out_b->size()==4,"new cloud processed once after empty epoch");
    check((actual.rot_end-reference.rot_end).norm()<1e-12,"future attitude unaffected by missing scan");
    check((actual.pos_end-reference.pos_end).norm()<1e-12,"future position unaffected by missing scan");
    check((actual.cov-reference.cov).norm()<1e-12,"future covariance unaffected by missing scan");
    for(size_t i=0;i<out_a->size();++i) {
      const auto& x=out_a->points[i];const auto& y=out_b->points[i];
      check(std::abs(x.x-y.x)+std::abs(x.y-y.y)+std::abs(x.z-y.z)<1e-7,"future cloud undistortion equals real-cloud control");
    }
    std::cout<<"PASS "<<checks<<" checks; actual ImuProcess empty-LIO and same-stamp VIO propagation\n";
    return 0;
  } catch(const std::exception& error) {
    std::cerr<<"FAIL after "<<checks<<" checks: "<<error.what()<<'\n';return 1;
  }
}
