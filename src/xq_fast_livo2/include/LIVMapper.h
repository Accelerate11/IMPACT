/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIV_MAPPER_H
#define LIV_MAPPER_H

#include "IMU_Processing.h"
#include "vio.h"
#include "preprocess.h"
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <image_transport/image_transport.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <vikit/camera_loader.h>
#include <unordered_set>
#include <map>
#include <array>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/u_int64_multi_array.hpp>
#include "research_joint_integration.h"

class LIVMapper
{
public:
  LIVMapper(rclcpp::Node::SharedPtr &node, std::string node_name);
  ~LIVMapper();
  void initializeSubscribersAndPublishers(rclcpp::Node::SharedPtr &nh, image_transport::ImageTransport &it_);
  void initializeComponents(rclcpp::Node::SharedPtr &node);
  void initializeFiles();
  void run(rclcpp::Node::SharedPtr &node);
  void gravityAlignment();
  void handleFirstFrame();
  void stateEstimationAndMapping();
  void handleVIO();
  void handleLIO();
  void savePCD();
  void processImu();
  
  bool sync_packages(LidarMeasureGroup &meas);
  void prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr);
  void imu_prop_callback();
  void transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud);
  void pointBodyToWorld(const PointType &pi, PointType &po);
  void RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po);
  void RGBpointBodyToWorld(PointType const *const pi, PointType *const po);
  void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
  void livox_pcl_cbk(const xq_livox_interfaces::msg::CustomMsg::ConstSharedPtr &msg_in);
  void imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr &msg_in);
  void img_cbk(const sensor_msgs::msg::Image::ConstSharedPtr &msg_in);
  void jointDownCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
  void jointRangeCallback(const sensor_msgs::msg::Range::ConstSharedPtr &msg);
  void jointBarrierCallback(const std_msgs::msg::UInt64MultiArray::ConstSharedPtr &msg);
  void processJointEpoch(int64_t stamp_ns);
  void publish_img_rgb(const image_transport::Publisher &pubImage, VIOManagerPtr vio_manager);
  void publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFullRes, VIOManagerPtr vio_manager);
  void publish_visual_sub_map(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubSubVisualMap);
  void publish_effect_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list);
  void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &pmavros_pose_publisherubOdomAftMapped);
  void publish_mavros(const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr &mavros_pose_publisher);
  void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &pubPath);
  void readParameters(rclcpp::Node::SharedPtr &node);
  void recordVisualUpdate(const StatesGroup &before, const StatesGroup *state_before, double image_stamp);
  void recordIngress(size_t sensor, int64_t stamp_ns);
  void writeIngressResult();
  void recordImageEpoch(uint64_t sequence, int64_t stamp_ns, const std::string &classification,
                        const std::string &reason, bool odometry_published, uint64_t lidar_points);
  void tryFinalizeProcessing();
  template <typename T> void set_posestamp(T &out);
  template <typename T> void pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po);
  template <typename T> Eigen::Matrix<T, 3, 1> pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi);
  cv::Mat getImageFromMsg(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg);

  std::mutex mtx_buffer, mtx_buffer_imu_prop;
  std::condition_variable sig_buffer;

  SLAM_MODE slam_mode_;
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map;
  
  string root_dir;
  bool research_visual_update_enabled = true, research_shadow_only = true;
  bool research_joint_inputs = false;
  std::unique_ptr<impact_research::JointIntegration> research_joint;
  std::map<int64_t, cv::Mat> joint_down_images;
  std::map<int64_t, impact_research::EpochRange> joint_ranges;
  std::map<int64_t, std::array<uint64_t,5>> joint_barriers;
  uint64_t joint_down_received = 0, joint_range_received = 0;
  int64_t joint_last_down_ns = -1, joint_last_range_ns = -1, joint_last_barrier_ns = -1;
  std::array<uint64_t,5> joint_last_barrier_counts = {0,0,0,0,0};
  int64_t expected_joint_down = -1, expected_joint_range = -1;
  int64_t expected_joint_last_down_ns = -1, expected_joint_last_range_ns = -1, expected_joint_last_barrier_ns = -1;
  string research_run_file, research_run_dir;
  string research_odom_topic = "/impact/r2b2/odom", research_map_frame = "impact_livo_map", research_body_frame = "livox_imu";
  ofstream research_visual_log, research_odometry_log;
  uint64_t research_epoch_index = 0;
  uint64_t ingress_counts[3] = {0, 0, 0};
  double ingress_first_stamp[3] = {-1.0, -1.0, -1.0};
  double ingress_last_stamp[3] = {-1.0, -1.0, -1.0};
  bool ingress_result_written = false;
  int64_t ingress_first_stamp_ns[3] = {-1, -1, -1}, ingress_last_stamp_ns[3] = {-1, -1, -1};
  int64_t expected_messages[3] = {-1, -1, -1};
  int64_t expected_last_eligible_image_stamp_ns = -1, expected_last_lidar_stamp_ns = -1, expected_last_imu_stamp_ns = -1;
  int64_t last_completed_post_vio_header_stamp_ns = -1;
  ofstream research_image_ledger;
  uint64_t visual_diagnostic_rows = 0, odometry_rows = 0;
  uint64_t image_classification_counts[4] = {0, 0, 0, 0};
  std::unordered_set<uint64_t> recorded_image_sequences;
  std::vector<std::pair<int64_t, size_t>> recorded_image_classifications;
  bool processing_result_written = false;
  uint64_t latest_lio_points = 0;
  double last_nonempty_lio_update_time = -1.0;
  string lid_topic, imu_topic, seq_name, img_topic;
  V3D extT;
  M3D extR;

  int feats_down_size = 0, max_iterations = 0;

  double res_mean_last = 0.05;
  double gyr_cov = 0, acc_cov = 0, inv_expo_cov = 0;
  double blind_rgb_points = 0.0;
  double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0, last_timestamp_img = -1.0;
  double filter_size_surf_min = 0;
  double filter_size_pcd = 0;
  double _first_lidar_time = 0.0;
  double match_time = 0, solve_time = 0, solve_const_H_time = 0;

  bool lidar_map_inited = false, pcd_save_en = false, img_save_en = false, pub_effect_point_en = false, pose_output_en = false, ros_driver_fix_en = false, hilti_en = false;
  int img_save_interval = 1, pcd_save_interval = -1, pcd_save_type = 0;
  int pub_scan_num = 1;

  StatesGroup imu_propagate, latest_ekf_state;

  bool new_imu = false, state_update_flg = false, imu_prop_enable = true, ekf_finish_once = false;
  deque<sensor_msgs::msg::Imu> prop_imu_buffer;
  sensor_msgs::msg::Imu newest_imu;
  double latest_ekf_time;
  nav_msgs::msg::Odometry imu_prop_odom;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubImuPropOdom;
  double imu_time_offset = 0.0;
  double lidar_time_offset = 0.0;

  bool gravity_align_en = false, gravity_align_finished = false;

  bool sync_jump_flag = false;

  bool lidar_pushed = false, imu_en, gravity_est_en, flg_reset = false, ba_bg_est_en = true;
  bool dense_map_en = false;
  int img_en = 1, imu_int_frame = 3;
  bool normal_en = true;
  bool exposure_estimate_en = false;
  double exposure_time_init = 0.0;
  bool inverse_composition_en = false;
  bool raycast_en = false;
  int lidar_en = 1;
  bool is_first_frame = false;
  int grid_size = 0, patch_size = 0, grid_n_width = 0, grid_n_height = 0, patch_pyrimid_level = 0;
  int outlier_threshold;
  double plot_time;
  int frame_cnt;
  double img_time_offset = 0.0;
  deque<PointCloudXYZI::Ptr> lid_raw_data_buffer;
  deque<double> lid_header_time_buffer;
  deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;
  deque<cv::Mat> img_buffer;
  deque<double> img_time_buffer;
  deque<double> img_header_time_buffer;
  deque<int64_t> img_header_stamp_ns_buffer;
  deque<uint64_t> img_sequence_buffer;
  vector<pointWithVar> _pv_list;
  vector<double> extrinT;
  vector<double> extrinR;
  vector<double> cameraextrinT;
  vector<double> cameraextrinR;
  int IMG_POINT_COV;

  PointCloudXYZI::Ptr visual_sub_map;
  PointCloudXYZI::Ptr feats_undistort;
  PointCloudXYZI::Ptr feats_down_body;
  PointCloudXYZI::Ptr feats_down_world;
  PointCloudXYZI::Ptr pcl_w_wait_pub;
  PointCloudXYZI::Ptr pcl_wait_pub;
  PointCloudXYZRGB::Ptr pcl_wait_save;
  PointCloudXYZI::Ptr pcl_wait_save_intensity;

  ofstream fout_pre, fout_out, fout_visual_pos, fout_lidar_pos, fout_points;

  pcl::VoxelGrid<PointType> downSizeFilterSurf;

  V3D euler_cur;

  LidarMeasureGroup LidarMeasures;
  StatesGroup _state;
  StatesGroup  state_propagat;

  nav_msgs::msg::Path path;
  nav_msgs::msg::Odometry odomAftMapped;
  geometry_msgs::msg::Quaternion geoQuat;
  geometry_msgs::msg::PoseStamped msg_body_pose;

  PreprocessPtr p_pre;
  ImuProcessPtr p_imu;
  VoxelMapManagerPtr voxelmap_manager;
  VIOManagerPtr vio_manager;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr plane_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr voxel_pub;
  std::shared_ptr<rclcpp::SubscriptionBase> sub_pcl;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_joint_down;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_joint_range;
  rclcpp::Subscription<std_msgs::msg::UInt64MultiArray>::SharedPtr sub_joint_barrier;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullRes;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubNormal;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSubVisualMap;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudDyn;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudDynRmed;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudDynDbg;
  image_transport::Publisher pubImage;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mavros_pose_publisher;
  rclcpp::TimerBase::SharedPtr imu_prop_timer;
  rclcpp::Node::SharedPtr node;

  int frame_num = 0;
  double aver_time_consu = 0;
  double aver_time_icp = 0;
  double aver_time_map_inre = 0;
  bool colmap_output_en = false;
};
#endif
