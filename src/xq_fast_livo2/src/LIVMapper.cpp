/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "LIVMapper.h"
#include <vikit/camera_loader.h>
#include <filesystem>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace
{
int64_t stampNanoseconds(const builtin_interfaces::msg::Time &stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}
const char *imageClassifications[] = {"processed_postVIO", "startup_discard", "imu_initialization", "terminal_uncovered"};
void jsonNumber(std::ostream &out, double value)
{
  if (std::isfinite(value)) out << std::setprecision(17) << value;
  else out << "null"; // Preserve invalid-state evidence without emitting invalid JSON.
}
template <typename Derived> void jsonMatrix(std::ostream &out, const Eigen::MatrixBase<Derived> &matrix)
{
  out << '[';
  for (Eigen::Index row = 0; row < matrix.rows(); ++row)
  {
    if (row) out << ',';
    out << '[';
    for (Eigen::Index column = 0; column < matrix.cols(); ++column)
    {
      if (column) out << ',';
      jsonNumber(out, matrix(row, column));
    }
    out << ']';
  }
  out << ']';
}
template <typename Derived> void jsonVector(std::ostream &out, const Eigen::MatrixBase<Derived> &vector)
{
  out << '[';
  for (Eigen::Index index = 0; index < vector.size(); ++index)
  {
    if (index) out << ',';
    jsonNumber(out, vector(index));
  }
  out << ']';
}
std::string jsonString(const std::string &value)
{
  std::ostringstream out;
  out << '"';
  for (unsigned char character : value)
  {
    if (character == '"' || character == '\\') out << '\\' << character;
    else if (character < 0x20) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << unsigned(character);
    else out << character;
  }
  out << '"';
  return out.str();
}
void jsonPose(std::ostream &out, const StatesGroup &state)
{
  Eigen::Quaterniond quaternion(state.rot_end);
  quaternion.normalize();
  out << "{\"position\":";
  jsonVector(out, state.pos_end);
  out << ",\"quaternion_xyzw\":";
  jsonVector(out, quaternion.coeffs());
  out << '}';
}
bool stateFinite(const StatesGroup &state)
{
  return state.rot_end.allFinite() && state.pos_end.allFinite() && state.vel_end.allFinite() &&
         state.bias_g.allFinite() && state.bias_a.allFinite() && state.gravity.allFinite() &&
         std::isfinite(state.inv_expo_time) && state.cov.allFinite();
}
Eigen::Matrix<double, 6, 6> poseCovariance(const StatesGroup &state)
{
  // StatesGroup uses R * Exp(dtheta_body), p_world + dp_world.
  Eigen::Matrix<double, 6, DIM_STATE> jacobian = Eigen::Matrix<double, 6, DIM_STATE>::Zero();
  jacobian.block<3, 3>(0, 3).setIdentity();
  jacobian.block<3, 3>(3, 0) = state.rot_end;
  return jacobian * state.cov * jacobian.transpose();
}
} // namespace

using namespace Sophus;
LIVMapper::LIVMapper(rclcpp::Node::SharedPtr &node, std::string node_name)
    : node(std::make_shared<rclcpp::Node>(node_name)),
      extT(0, 0, 0),
      extR(M3D::Identity())
{
  extrinT.assign(3, 0.0);
  extrinR.assign(9, 0.0);
  cameraextrinT.assign(3, 0.0);
  cameraextrinR.assign(9, 0.0);
  
  p_pre.reset(new Preprocess());
  p_imu.reset(new ImuProcess());

  readParameters(this->node);
  VoxelMapConfig voxel_config;
  loadVoxelConfig(this->node, voxel_config);

  visual_sub_map.reset(new PointCloudXYZI());
  feats_undistort.reset(new PointCloudXYZI());
  feats_down_body.reset(new PointCloudXYZI());
  feats_down_world.reset(new PointCloudXYZI());
  pcl_w_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_save.reset(new PointCloudXYZRGB());
  pcl_wait_save_intensity.reset(new PointCloudXYZI());
  voxelmap_manager.reset(new VoxelMapManager(voxel_config, voxel_map));
  vio_manager.reset(new VIOManager());
  initializeFiles();
  initializeComponents(this->node);          // initialize components errors
  path.header.stamp = this->node->now();
  path.header.frame_id = research_map_frame;
}

LIVMapper::~LIVMapper()
{
  try { writeIngressResult(); }
  catch (const std::exception &error) { std::cerr << "Ingress result write failed: " << error.what() << std::endl; }
}

void LIVMapper::readParameters(rclcpp::Node::SharedPtr &node)
{
  research_visual_update_enabled = this->node->declare_parameter<bool>("research.visual_update_enabled", true);
  research_joint_inputs = this->node->declare_parameter<bool>("research.joint_inputs", false);
  expected_joint_down = this->node->declare_parameter<int64_t>("research.expected_down_messages", -1);
  expected_joint_range = this->node->declare_parameter<int64_t>("research.expected_range_messages", -1);
  expected_joint_last_down_ns = this->node->declare_parameter<int64_t>("research.last_down_stamp_ns", -1);
  expected_joint_last_range_ns = this->node->declare_parameter<int64_t>("research.last_range_stamp_ns", -1);
  expected_joint_last_barrier_ns = this->node->declare_parameter<int64_t>("research.last_barrier_stamp_ns", -1);
  if (research_joint_inputs && (expected_joint_down <= 0 || expected_joint_range <= 0 || expected_joint_last_down_ns <= 0 ||
      expected_joint_last_range_ns <= 0 || expected_joint_last_barrier_ns < std::max(expected_joint_last_down_ns, expected_joint_last_range_ns)))
    throw std::invalid_argument("Joint replay requires all five sensor counts, acquisition tails and final barrier contract");
  research_shadow_only = this->node->declare_parameter<bool>("research.shadow_only", true);
  research_run_file = this->node->declare_parameter<std::string>("research.run_file", "");
  research_run_dir = this->node->declare_parameter<std::string>("research.run_dir", "");
  research_odom_topic = this->node->declare_parameter<std::string>("research.odom_topic", "/impact/r2b2/odom");
  research_map_frame = this->node->declare_parameter<std::string>("research.map_frame", "impact_livo_map");
  research_body_frame = this->node->declare_parameter<std::string>("research.body_frame", "livox_imu");
  expected_messages[0] = this->node->declare_parameter<int64_t>("research.expected_lidar_messages", -1);
  expected_messages[1] = this->node->declare_parameter<int64_t>("research.expected_imu_messages", -1);
  expected_messages[2] = this->node->declare_parameter<int64_t>("research.expected_image_messages", -1);
  expected_last_eligible_image_stamp_ns = this->node->declare_parameter<int64_t>("research.last_eligible_image_stamp_ns", -1);
  expected_last_lidar_stamp_ns = this->node->declare_parameter<int64_t>("research.last_lidar_stamp_ns", -1);
  expected_last_imu_stamp_ns = this->node->declare_parameter<int64_t>("research.last_imu_stamp_ns", -1);
  if (!research_shadow_only) throw std::invalid_argument("This research build permits shadow-only operation");
  if (expected_messages[0] <= 0 || expected_messages[1] <= 0 || expected_messages[2] <= 0 ||
      expected_last_eligible_image_stamp_ns <= 0 || expected_last_lidar_stamp_ns <= 0 || expected_last_imu_stamp_ns <= 0 ||
      expected_last_eligible_image_stamp_ns > std::min(expected_last_lidar_stamp_ns, expected_last_imu_stamp_ns) || research_run_file.empty())
    throw std::invalid_argument("Frozen positive expected input counts/tail stamps and research.run_file are required");
  // declare parameters
  this->node->declare_parameter<std::string>("common.lid_topic", "/livox/lidar");
  this->node->declare_parameter<std::string>("common.imu_topic", "/livox/imu");
  this->node->declare_parameter<bool>("common.ros_driver_bug_fix", false);
  this->node->declare_parameter<int>("common.img_en", 1);
  this->node->declare_parameter<int>("common.lidar_en", 1);
  this->node->declare_parameter<std::string>("common.img_topic", "/left_camera/image");

  this->node->declare_parameter<bool>("vio.normal_en", true);
  this->node->declare_parameter<bool>("vio.inverse_composition_en", false);
  this->node->declare_parameter<int>("vio.max_iterations", 5);
  this->node->declare_parameter<int>("vio.img_point_cov", 100);
  this->node->declare_parameter<bool>("vio.raycast_en", false);
  this->node->declare_parameter<bool>("vio.exposure_estimate_en", true);
  this->node->declare_parameter<double>("vio.inv_expo_cov", 0.1);
  this->node->declare_parameter<int>("vio.grid_size", 5);
  this->node->declare_parameter<int>("vio.grid_n_height", 17);
  this->node->declare_parameter<int>("vio.patch_pyrimid_level", 4);
  this->node->declare_parameter<int>("vio.patch_size", 8);
  this->node->declare_parameter<int>("vio.outlier_threshold", 100);
  this->node->declare_parameter<double>("time_offset.exposure_time_init", 0.0);
  this->node->declare_parameter<double>("time_offset.img_time_offset", 0.0);
  this->node->declare_parameter<double>("time_offset.imu_time_offset", 0.0);
  this->node->declare_parameter<double>("time_offset.lidar_time_offset", 0.0);
  this->node->declare_parameter<bool>("uav.imu_rate_odom", false);
  this->node->declare_parameter<bool>("uav.gravity_align_en", false);

  this->node->declare_parameter<std::string>("evo.seq_name", "01");
  this->node->declare_parameter<bool>("evo.pose_output_en", false);
  this->node->declare_parameter<double>("imu.gyr_cov", 1.0);
  this->node->declare_parameter<double>("imu.acc_cov", 1.0);
  this->node->declare_parameter<int>("imu.imu_int_frame", 30);
  this->node->declare_parameter<bool>("imu.imu_en", true);
  this->node->declare_parameter<bool>("imu.gravity_est_en", true);
  this->node->declare_parameter<bool>("imu.ba_bg_est_en", true);

  this->node->declare_parameter<double>("preprocess.blind", 0.01);
    this->node->declare_parameter<bool>("preprocess.hilti_en", false);
  this->node->declare_parameter<double>("preprocess.filter_size_surf", 0.5);
  this->node->declare_parameter<int>("preprocess.lidar_type", AVIA);
  this->node->declare_parameter<int>("preprocess.scan_line",6);
  this->node->declare_parameter<int>("preprocess.point_filter_num", 3);
  this->node->declare_parameter<bool>("preprocess.feature_extract_enabled", false);

  this->node->declare_parameter<int>("pcd_save.interval", -1);
  this->node->declare_parameter<bool>("pcd_save.pcd_save_en", false);
  this->node->declare_parameter<bool>("image_save.img_save_en", false);
  this->node->declare_parameter<int>("image_save.interval", 1);

  this->node->declare_parameter<int>("pcd_save.type", 0);
  this->node->declare_parameter<bool>("pcd_save.colmap_output_en", false);
  this->node->declare_parameter<double>("pcd_save.filter_size_pcd", 0.5);
  this->node->declare_parameter<vector<double>>("extrin_calib.extrinsic_T", vector<double>{});
  this->node->declare_parameter<vector<double>>("extrin_calib.extrinsic_R", vector<double>{});
  this->node->declare_parameter<vector<double>>("extrin_calib.Pcl", vector<double>{});
  this->node->declare_parameter<vector<double>>("extrin_calib.Rcl", vector<double>{});
  this->node->declare_parameter<double>("debug.plot_time", -10);
  this->node->declare_parameter<int>("debug.frame_cnt", 6);

  this->node->declare_parameter<double>("publish.blind_rgb_points", 0.01);
  this->node->declare_parameter<int>("publish.pub_scan_num", 1);
  this->node->declare_parameter<bool>("publish.pub_effect_point_en", false);
  this->node->declare_parameter<bool>("publish.dense_map_en", false);

  // get parameter
  this->node->get_parameter("common.lid_topic", lid_topic);
  this->node->get_parameter("common.imu_topic", imu_topic);
  this->node->get_parameter("common.ros_driver_bug_fix", ros_driver_fix_en);
  this->node->get_parameter("common.img_en", img_en);
  this->node->get_parameter("common.lidar_en", lidar_en);
  this->node->get_parameter("common.img_topic", img_topic);

  this->node->get_parameter("vio.normal_en", normal_en);
  this->node->get_parameter("vio.inverse_composition_en", inverse_composition_en);
  this->node->get_parameter("vio.max_iterations", max_iterations);
  this->node->get_parameter("vio.img_point_cov", IMG_POINT_COV);
  this->node->get_parameter("vio.raycast_en", raycast_en);
  this->node->get_parameter("vio.exposure_estimate_en", exposure_estimate_en);
  this->node->get_parameter("vio.inv_expo_cov", inv_expo_cov);
  this->node->get_parameter("vio.grid_size", grid_size);
  this->node->get_parameter("vio.grid_n_height", grid_n_height);
  this->node->get_parameter("vio.patch_pyrimid_level", patch_pyrimid_level);
  this->node->get_parameter("vio.patch_size", patch_size);
  this->node->get_parameter("vio.outlier_threshold", outlier_threshold);
  this->node->get_parameter("time_offset.exposure_time_init", exposure_time_init);
  this->node->get_parameter("time_offset.img_time_offset", img_time_offset);
  this->node->get_parameter("time_offset.imu_time_offset", imu_time_offset);
  this->node->get_parameter("time_offset.lidar_time_offset", lidar_time_offset);
  this->node->get_parameter("uav.imu_rate_odom", imu_prop_enable);
  this->node->get_parameter("uav.gravity_align_en", gravity_align_en);

  this->node->get_parameter("evo.seq_name", seq_name);
  this->node->get_parameter("evo.pose_output_en", pose_output_en);
  this->node->get_parameter("imu.gyr_cov", gyr_cov);
  this->node->get_parameter("imu.acc_cov", acc_cov);
  this->node->get_parameter("imu.imu_int_frame", imu_int_frame);
  this->node->get_parameter("imu.imu_en", imu_en);
  this->node->get_parameter("imu.gravity_est_en", gravity_est_en);
  this->node->get_parameter("imu.ba_bg_est_en", ba_bg_est_en);

  this->node->get_parameter("preprocess.blind", p_pre->blind);
  this->node->get_parameter("preprocess.filter_size_surf", filter_size_surf_min);
  this->node->get_parameter("preprocess.lidar_type", p_pre->lidar_type);
  this->node->get_parameter("preprocess.scan_line", p_pre->N_SCANS);
  this->node->get_parameter("preprocess.point_filter_num", p_pre->point_filter_num);
  this->node->get_parameter("preprocess.feature_extract_enabled", p_pre->feature_enabled);

  this->node->get_parameter("pcd_save.interval", pcd_save_interval);
  this->node->get_parameter("pcd_save.pcd_save_en", pcd_save_en);
  this->node->get_parameter("image_save.img_save_en", img_save_en);
  this->node->get_parameter("pcd_save.colmap_output_en", colmap_output_en);
  this->node->get_parameter("pcd_save.filter_size_pcd", filter_size_pcd);
  this->node->get_parameter("extrin_calib.extrinsic_T", extrinT);
  this->node->get_parameter("extrin_calib.extrinsic_R", extrinR);
  this->node->get_parameter("extrin_calib.Pcl", cameraextrinT);
  this->node->get_parameter("extrin_calib.Rcl", cameraextrinR);
  this->node->get_parameter("debug.plot_time", plot_time);
  this->node->get_parameter("debug.frame_cnt", frame_cnt);

  this->node->get_parameter("publish.blind_rgb_points", blind_rgb_points);
  this->node->get_parameter("publish.pub_scan_num", pub_scan_num);
  this->node->get_parameter("publish.pub_effect_point_en", pub_effect_point_en);
  this->node->get_parameter("publish.dense_map_en", dense_map_en);
  if (pcd_save_en || img_save_en || colmap_output_en || pose_output_en)
    throw std::invalid_argument("Upstream PCD/image/Colmap/pose exporters are disabled in the isolated research build");
  if (extrinT.size() != 3 || extrinR.size() != 9 || cameraextrinT.size() != 3 || cameraextrinR.size() != 9)
    throw std::invalid_argument("Explicit LiDAR/IMU/camera extrinsics are required");
}

void LIVMapper::initializeComponents(rclcpp::Node::SharedPtr &node) 
{
  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
  
  // extrinT.assign({0.04165, 0.02326, -0.0284});
  // extrinR.assign({1, 0, 0, 0, 1, 0, 0, 0, 1});
  // cameraextrinT.assign({0.0194384, 0.104689,-0.0251952});
  // cameraextrinR.assign({0.00610193,-0.999863,-0.0154172,-0.00615449,0.0153796,-0.999863,0.999962,0.00619598,-0.0060598});

  extT << VEC_FROM_ARRAY(extrinT);
  extR << MAT_FROM_ARRAY(extrinR);

  voxelmap_manager->extT_ << VEC_FROM_ARRAY(extrinT);
  voxelmap_manager->extR_ << MAT_FROM_ARRAY(extrinR);

  if (!vk::camera_loader::loadFromRosNs(this->node, "parameter_blackboard", vio_manager->cam)) throw std::runtime_error("Camera model not correctly specified.");

  vio_manager->grid_size = grid_size;
  vio_manager->patch_size = patch_size;
  vio_manager->outlier_threshold = outlier_threshold;
  vio_manager->setImuToLidarExtrinsic(extT, extR);
  vio_manager->setLidarToCameraExtrinsic(cameraextrinR, cameraextrinT);
  vio_manager->state = &_state;
  vio_manager->state_propagat = &state_propagat;
  vio_manager->max_iterations = max_iterations;
  vio_manager->img_point_cov = IMG_POINT_COV;
  vio_manager->normal_en = normal_en;
  vio_manager->inverse_composition_en = inverse_composition_en;
  vio_manager->raycast_en = raycast_en;
  vio_manager->grid_n_width = grid_n_width;
  vio_manager->grid_n_height = grid_n_height;
  vio_manager->patch_pyrimid_level = patch_pyrimid_level;
  vio_manager->exposure_estimate_en = exposure_estimate_en;
  vio_manager->colmap_output_en = colmap_output_en;
  vio_manager->visual_update_enabled = research_visual_update_enabled;
  vio_manager->initializeVIO();

  p_imu->set_extrinsic(extT, extR);
  p_imu->set_gyr_cov_scale(V3D(gyr_cov, gyr_cov, gyr_cov));
  p_imu->set_acc_cov_scale(V3D(acc_cov, acc_cov, acc_cov));
  p_imu->set_inv_expo_cov(inv_expo_cov);
  p_imu->set_gyr_bias_cov(V3D(0.0001, 0.0001, 0.0001));
  p_imu->set_acc_bias_cov(V3D(0.0001, 0.0001, 0.0001));
  p_imu->set_imu_init_frame_num(imu_int_frame);

  if (!imu_en) p_imu->disable_imu();
  if (!gravity_est_en) p_imu->disable_gravity_est();
  if (!ba_bg_est_en) p_imu->disable_bias_est();
  if (!exposure_estimate_en) p_imu->disable_exposure_est();

  slam_mode_ = (img_en && lidar_en) ? LIVO : imu_en ? ONLY_LIO : ONLY_LO;
}

void LIVMapper::initializeFiles() 
{
  namespace fs = std::filesystem;
  fast_livo_runtime_root().clear();
  root_dir.clear();
  if (!research_run_dir.empty())
  {
    const fs::path directory(research_run_dir);
    if (!directory.is_absolute()) throw std::invalid_argument("research.run_dir must be absolute");
    fs::create_directories(directory / "Log");
    root_dir = directory.lexically_normal().string();
    fast_livo_runtime_root() = root_dir;
  }
  if (!research_run_file.empty())
  {
    const fs::path visual_file(research_run_file);
    if (!visual_file.is_absolute()) throw std::invalid_argument("research.run_file must be absolute");
    const fs::path odometry_file = visual_file.parent_path() / "odometry.jsonl";
    const fs::path ledger_file = visual_file.parent_path() / "image-epochs.jsonl";
    if (visual_file == odometry_file || visual_file == ledger_file || fs::exists(visual_file) || fs::exists(odometry_file) || fs::exists(ledger_file) ||
        fs::exists(visual_file.parent_path() / "processing-result.json") || fs::exists(visual_file.parent_path() / "processing-result.json.tmp"))
      throw std::invalid_argument("Research JSONL files must be new, distinct files");
    fs::create_directories(visual_file.parent_path());
    research_visual_log.open(visual_file);
    research_odometry_log.open(odometry_file);
    research_image_ledger.open(ledger_file);
    if (!research_visual_log || !research_odometry_log || !research_image_ledger) throw std::runtime_error("Cannot open research JSONL files");
  }
  fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), std::ios::out);
  fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), std::ios::out);
  if (research_joint_inputs)
  {
    if (research_run_dir.empty()) throw std::invalid_argument("Joint research needs an absolute run directory");
    research_joint = std::make_unique<impact_research::JointIntegration>(this->node, research_run_dir);
  }
}

void LIVMapper::initializeSubscribersAndPublishers(rclcpp::Node::SharedPtr &node, image_transport::ImageTransport &it_)
{
  image_transport::ImageTransport it(this->node);
  // This isolated package is a shadow replay consumer. Match the frozen
  // reliable replay publisher so fragment loss is recovered and exact input
  // accounting can be verified. This is not a hardware QoS/timing claim.
  const auto replay_qos = rclcpp::QoS(rclcpp::KeepLast(1000)).reliable();
  if (p_pre->lidar_type == AVIA) {
    sub_pcl = this->node->create_subscription<xq_livox_interfaces::msg::CustomMsg>(lid_topic, replay_qos, std::bind(&LIVMapper::livox_pcl_cbk, this, std::placeholders::_1));
  } else {
    sub_pcl = this->node->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, replay_qos, std::bind(&LIVMapper::standard_pcl_cbk, this, std::placeholders::_1));
  }
  sub_imu = this->node->create_subscription<sensor_msgs::msg::Imu>(imu_topic, replay_qos, std::bind(&LIVMapper::imu_cbk, this, std::placeholders::_1));
  sub_img = this->node->create_subscription<sensor_msgs::msg::Image>(img_topic, replay_qos, std::bind(&LIVMapper::img_cbk, this, std::placeholders::_1));
  if (research_joint_inputs)
  {
    sub_joint_down = this->node->create_subscription<sensor_msgs::msg::Image>("/impact/joint/input/down", replay_qos,
        std::bind(&LIVMapper::jointDownCallback, this, std::placeholders::_1));
    sub_joint_range = this->node->create_subscription<sensor_msgs::msg::Range>("/impact/joint/input/range", replay_qos,
        std::bind(&LIVMapper::jointRangeCallback, this, std::placeholders::_1));
    sub_joint_barrier = this->node->create_subscription<std_msgs::msg::UInt64MultiArray>("/impact/joint/input/barrier", replay_qos,
        std::bind(&LIVMapper::jointBarrierCallback, this, std::placeholders::_1));
  }
  
  pubLaserCloudFullRes = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 100);
  pubNormal = this->node->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_marker", 100);
  pubSubVisualMap = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_visual_sub_map_before", 100);
  pubLaserCloudEffect = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 100);
  pubLaserCloudMap = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 100);
  pubOdomAftMapped = this->node->create_publisher<nav_msgs::msg::Odometry>(research_odom_topic, 100);
  pubPath = this->node->create_publisher<nav_msgs::msg::Path>("/path", 10);
  plane_pub = this->node->create_publisher<visualization_msgs::msg::Marker>("/planner_normal", 1);
  voxel_pub = this->node->create_publisher<visualization_msgs::msg::MarkerArray>("/voxels", 1);
  pubLaserCloudDyn = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/dyn_obj", 100);
  pubLaserCloudDynRmed = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/dyn_obj_removed", 100);
  pubLaserCloudDynDbg = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/dyn_obj_dbg_hist", 100);
  // Shadow estimator: deliberately create no MAVROS publisher or TF broadcaster.
  pubImage = it.advertise("/rgb_img", 1);
  pubImuPropOdom = this->node->create_publisher<nav_msgs::msg::Odometry>("/LIVO2/imu_propagate", 10000);
  imu_prop_timer = this->node->create_wall_timer(0.004s, std::bind(&LIVMapper::imu_prop_callback, this));
  voxelmap_manager->voxel_map_pub_= this->node->create_publisher<visualization_msgs::msg::MarkerArray>("/planes", 10000);
}

void LIVMapper::handleFirstFrame() 
{
  if (!is_first_frame)
  {
    _first_lidar_time = LidarMeasures.last_lio_update_time;
    p_imu->first_lidar_time = _first_lidar_time; // Only for IMU data log
    is_first_frame = true;
    cout << "FIRST LIDAR FRAME!" << endl;
  }
}

void LIVMapper::gravityAlignment() 
{
  if (!p_imu->imu_need_init && !gravity_align_finished) 
  {
    std::cout << "Gravity Alignment Starts" << std::endl;
    V3D ez(0, 0, -1), gz(_state.gravity);
    Eigen::Quaterniond G_q_I0 = Eigen::Quaterniond::FromTwoVectors(gz, ez);
    M3D G_R_I0 = G_q_I0.toRotationMatrix();

    _state.pos_end = G_R_I0 * _state.pos_end;
    _state.rot_end = G_R_I0 * _state.rot_end;
    _state.vel_end = G_R_I0 * _state.vel_end;
    _state.gravity = G_R_I0 * _state.gravity;
    gravity_align_finished = true;
    std::cout << "Gravity Alignment Finished" << std::endl;
  }
}

void LIVMapper::processImu() 
{
  // double t0 = omp_get_wtime();

  p_imu->Process2(LidarMeasures, _state, feats_undistort);

  if (research_joint)
  {
    if (gravity_align_en) throw std::invalid_argument("Joint research requires fixed initialized world axes (gravity_align_en=false)");
    research_joint->propagated(_state, p_imu->research_transition, p_imu->research_process_noise,
                               p_imu->research_gyro_process_noise, !p_imu->imu_need_init,
                               static_cast<int64_t>(std::llround(LidarMeasures.last_lio_update_time * 1e9)));
  }

  if (gravity_align_en) gravityAlignment();

  state_propagat = _state;
  voxelmap_manager->state_ = _state;
  voxelmap_manager->feats_undistort_ = feats_undistort;

  // double t_prop = omp_get_wtime();

  // std::cout << "[ Mapping ] feats_undistort: " << feats_undistort->size() << std::endl;
  // std::cout << "[ Mapping ] predict cov: " << _state.cov.diagonal().transpose() << std::endl;
  // std::cout << "[ Mapping ] predict sta: " << state_propagat.pos_end.transpose() << state_propagat.vel_end.transpose() << std::endl;
}

void LIVMapper::stateEstimationAndMapping() 
{
  switch (LidarMeasures.lio_vio_flg) 
  {
    case VIO:
      handleVIO();
      break;
    case LIO:
    case LO:
      handleLIO();
      break;
  }
}

void LIVMapper::handleVIO() 
{
  // No pose epoch is available until IMU initialization has completed. The
  // LIVO synchronizer still consumes the same inputs in both arms.
  const auto &measurement = LidarMeasures.measures.back();
  if (latest_lio_points == 0 && research_epoch_index > 0 && !research_joint_inputs)
    throw std::runtime_error("LIVO epoch has no processed LiDAR cloud after the first post-VIO output");
  // The opt-in five-input replay retains the original 15 Hz cameras and 10 Hz
  // LiDAR snapshots. Some camera epochs have no new LiDAR points. IMU still
  // propagates to the camera epoch; an existing map supports VIO without
  // fabricating/reusing a scan as a new LIO measurement.
  const bool existing_joint_map = research_joint_inputs && lidar_map_inited;
  if (p_imu->imu_need_init || (latest_lio_points == 0 && !existing_joint_map))
  {
    // Process2 returns immediately from its initialization branch, including
    // the call which clears imu_need_init. That call has not produced a cloud
    // for LIO, so it cannot yet support a post-LIO/VIO odometry output.
    const std::string reason = p_imu->imu_need_init ? "imu_initialization_incomplete" : "initialization_returned_without_lidar_cloud";
    recordImageEpoch(measurement.image_sequence_id, measurement.image_header_stamp_ns, "imu_initialization", reason, false, 0);
    return;
  }
  const StatesGroup before(_state);
  const StatesGroup *state_before = vio_manager->state;
  // No image/patch export in this isolated replay. Both arms still construct,
  // select and update the visual map; only the EKF correction is ablated.
  vio_manager->plot_flag = false;
  vio_manager->processFrame(LidarMeasures.measures.back().img, _pv_list, voxelmap_manager->voxel_map_, LidarMeasures.last_lio_update_time - _first_lidar_time);
  recordVisualUpdate(before, state_before, LidarMeasures.measures.back().image_header_time);
  if (research_joint)
  {
    research_joint->native_update(before, _state, "VIO", measurement.image_header_stamp_ns);
    processJointEpoch(measurement.image_header_stamp_ns);
    voxelmap_manager->state_ = _state;
  }
  publish_odometry(pubOdomAftMapped);
  recordImageEpoch(measurement.image_sequence_id, measurement.image_header_stamp_ns, "processed_postVIO",
                   "shared_state_epoch_completed", true, latest_lio_points);

  if (imu_prop_enable) 
  {
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }

  if (!research_joint_inputs || latest_lio_points > 0) publish_frame_world(pubLaserCloudFullRes, vio_manager);
  publish_img_rgb(pubImage, vio_manager);

  euler_cur = RotMtoEuler(_state.rot_end);
  fout_out << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " " << feats_undistort->points.size() << std::endl;
}

void LIVMapper::handleLIO() 
{    
  latest_lio_points = feats_undistort ? feats_undistort->size() : 0;
  euler_cur = RotMtoEuler(_state.rot_end);
  fout_pre << setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
           << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
           << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << endl;
           
  if (!feats_undistort || feats_undistort->empty())
  {
    std::cout << "[ LIO ]: No point!!!" << std::endl;
    if (research_joint)
      research_joint->input("lio_snapshot", static_cast<int64_t>(std::llround(LidarMeasures.last_lio_update_time * 1e9)),
                            false, "no_new_snapshot_skip_lio_measurement_keep_existing_map");
    return;
  }

  double t0 = omp_get_wtime();

  downSizeFilterSurf.setInputCloud(feats_undistort);
  downSizeFilterSurf.filter(*feats_down_body);
  
  double t_down = omp_get_wtime();

  feats_down_size = feats_down_body->points.size();
  voxelmap_manager->feats_down_body_ = feats_down_body;
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, feats_down_world);
  voxelmap_manager->feats_down_world_ = feats_down_world;
  voxelmap_manager->feats_down_size_ = feats_down_size;
  
  if (!lidar_map_inited) 
  {
    lidar_map_inited = true;
    voxelmap_manager->BuildVoxelMap();
  }

  double t1 = omp_get_wtime();

  const StatesGroup native_before(_state);
  voxelmap_manager->StateEstimation(state_propagat);
  _state = voxelmap_manager->state_;
  last_nonempty_lio_update_time = LidarMeasures.last_lio_update_time;
  if (research_joint)
  {
    research_joint->native_update(native_before, _state, "LIO", static_cast<int64_t>(std::llround(LidarMeasures.last_lio_update_time * 1e9)));
    voxelmap_manager->state_ = _state;
  }
  _pv_list = voxelmap_manager->pv_list_;

  double t2 = omp_get_wtime();

  if (imu_prop_enable) 
  {
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }

  if (pose_output_en) 
  {
    static bool pos_opend = false;
    static int ocount = 0;
    std::ofstream outFile, evoFile;
    if (!pos_opend) 
    {
      evoFile.open(fast_livo_debug_file("result/" + seq_name + ".txt"), std::ios::out);
      pos_opend = true;
      if (!evoFile.is_open()) RCLCPP_ERROR(this->node->get_logger(), "open fail\n");
    } 
    else 
    {
      evoFile.open(fast_livo_debug_file("result/" + seq_name + ".txt"), std::ios::app);
      if (!evoFile.is_open()) RCLCPP_ERROR(this->node->get_logger(), "open fail\n");
    }
    Eigen::Matrix4d outT;
    Eigen::Quaterniond q(_state.rot_end);
    evoFile << std::fixed;
    evoFile << LidarMeasures.last_lio_update_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
  }
  
  euler_cur = RotMtoEuler(_state.rot_end);
  geoQuat = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));
  if (slam_mode_ != LIVO) publish_odometry(pubOdomAftMapped);

  double t3 = omp_get_wtime();

  PointCloudXYZI::Ptr world_lidar(new PointCloudXYZI());
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, world_lidar);
  for (size_t i = 0; i < world_lidar->points.size(); i++) 
  {
    voxelmap_manager->pv_list_[i].point_w << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;
    M3D point_crossmat = voxelmap_manager->cross_mat_list_[i];
    M3D var = voxelmap_manager->body_cov_list_[i];
    var = (_state.rot_end * extR) * var * (_state.rot_end * extR).transpose() +
          (-point_crossmat) * _state.cov.block<3, 3>(0, 0) * (-point_crossmat).transpose() + _state.cov.block<3, 3>(3, 3);
    voxelmap_manager->pv_list_[i].var = var;
  }
  voxelmap_manager->UpdateVoxelMap(voxelmap_manager->pv_list_);
  std::cout << "[ LIO ] Update Voxel Map" << std::endl;
  _pv_list = voxelmap_manager->pv_list_;
  
  double t4 = omp_get_wtime();

  if(voxelmap_manager->config_setting_.map_sliding_en)
  {
    voxelmap_manager->mapSliding();
  }
  
  PointCloudXYZI::Ptr laserCloudFullRes(dense_map_en ? feats_undistort : feats_down_body);
  int size = laserCloudFullRes->points.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

  for (int i = 0; i < size; i++) 
  {
    RGBpointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
  }
  *pcl_w_wait_pub = *laserCloudWorld;

  publish_frame_world(pubLaserCloudFullRes, vio_manager);
  if (pub_effect_point_en) publish_effect_world(pubLaserCloudEffect, voxelmap_manager->ptpl_list_);
  if (voxelmap_manager->config_setting_.is_pub_plane_map_) voxelmap_manager->pubVoxelMap();
  if (slam_mode_ != LIVO) publish_path(pubPath);

  frame_num++;
  aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t4 - t0) / frame_num;

  // aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + (t2 - t1) / frame_num;
  // aver_time_map_inre = aver_time_map_inre * (frame_num - 1) / frame_num + (t4 - t3) / frame_num;
  // aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + (solve_time) / frame_num;
  // aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1) / frame_num + solve_const_H_time / frame_num;
  // printf("[ mapping time ]: per scan: propagation %0.6f downsample: %0.6f match: %0.6f solve: %0.6f  ICP: %0.6f  map incre: %0.6f total: %0.6f \n"
  //         "[ mapping time ]: average: icp: %0.6f construct H: %0.6f, total: %0.6f \n",
  //         t_prop - t0, t1 - t_prop, match_time, solve_time, t3 - t1, t5 - t3, t5 - t0, aver_time_icp, aver_time_const_H_time, aver_time_consu);

  // printf("\033[1;36m[ LIO mapping time ]: current scan: icp: %0.6f secs, map incre: %0.6f secs, total: %0.6f secs.\033[0m\n"
  //         "\033[1;36m[ LIO mapping time ]: average: icp: %0.6f secs, map incre: %0.6f secs, total: %0.6f secs.\033[0m\n",
  //         t2 - t1, t4 - t3, t4 - t0, aver_time_icp, aver_time_map_inre, aver_time_consu);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;34m|                         LIO Mapping Time                    |\033[0m\n");
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;34m| %-29s | %-27s |\033[0m\n", "Algorithm Stage", "Time (secs)");
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "DownSample", t_down - t0);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "ICP", t2 - t1);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "updateVoxelMap", t4 - t3);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Current Total Time", t4 - t0);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Average Total Time", aver_time_consu);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");

  euler_cur = RotMtoEuler(_state.rot_end);
  fout_out << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " " << feats_undistort->points.size() << std::endl;
}

void LIVMapper::savePCD() 
{
  if (pcd_save_en && (pcl_wait_save->points.size() > 0 || pcl_wait_save_intensity->points.size() > 0) && pcd_save_interval < 0) 
  {
    std::string raw_points_dir = fast_livo_debug_file("pcd/all_raw_points.pcd");
    std::string downsampled_points_dir = fast_livo_debug_file("pcd/all_downsampled_points.pcd");
    pcl::PCDWriter pcd_writer;

    if (img_en)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
      voxel_filter.setInputCloud(pcl_wait_save);
      voxel_filter.setLeafSize(filter_size_pcd, filter_size_pcd, filter_size_pcd);
      voxel_filter.filter(*downsampled_cloud);
  
      pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save); // Save the raw point cloud data
      std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir 
                << " with point count: " << pcl_wait_save->points.size() << RESET << std::endl;
      
      pcd_writer.writeBinary(downsampled_points_dir, *downsampled_cloud); // Save the downsampled point cloud data
      std::cout << GREEN << "Downsampled point cloud data saved to: " << downsampled_points_dir 
                << " with point count after filtering: " << downsampled_cloud->points.size() << RESET << std::endl;

      if(colmap_output_en)
      {
        fout_points << "# 3D point list with one line of data per point\n";
        fout_points << "#  POINT_ID, X, Y, Z, R, G, B, ERROR\n";
        for (size_t i = 0; i < downsampled_cloud->size(); ++i) 
        {
            const auto& point = downsampled_cloud->points[i];
            fout_points << i << " "
                        << std::fixed << std::setprecision(6)
                        << point.x << " " << point.y << " " << point.z << " "
                        << static_cast<int>(point.r) << " "
                        << static_cast<int>(point.g) << " "
                        << static_cast<int>(point.b) << " "
                        << 0 << std::endl;
        }
      }
    }
    else
    {      
      pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save_intensity);
      std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir 
                << " with point count: " << pcl_wait_save_intensity->points.size() << RESET << std::endl;
    }
  }
}

void LIVMapper::run(rclcpp::Node::SharedPtr &node) 
{
  rclcpp::WallRate rate(5000);
  while (rclcpp::ok()) 
  {
    rclcpp::spin_some(this->node);
    if (!rclcpp::ok()) break;
    if (!sync_packages(LidarMeasures)) 
    {
      tryFinalizeProcessing();
      rate.sleep();
      continue;
    }
    handleFirstFrame();

    processImu();

    // if (!p_imu->imu_time_init) continue;

    stateEstimationAndMapping();
    tryFinalizeProcessing();
  }
  savePCD();
  writeIngressResult();
}

void LIVMapper::prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr)
{
  double mean_acc_norm = p_imu->IMU_mean_acc_norm;
  acc_avr = acc_avr * G_m_s2 / mean_acc_norm - imu_prop_state.bias_a;
  angvel_avr -= imu_prop_state.bias_g;

  M3D Exp_f = Exp(angvel_avr, dt);
  /* propogation of IMU attitude */
  imu_prop_state.rot_end = imu_prop_state.rot_end * Exp_f;

  /* Specific acceleration (global frame) of IMU */
  V3D acc_imu = imu_prop_state.rot_end * acc_avr + V3D(imu_prop_state.gravity[0], imu_prop_state.gravity[1], imu_prop_state.gravity[2]);

  /* propogation of IMU */
  imu_prop_state.pos_end = imu_prop_state.pos_end + imu_prop_state.vel_end * dt + 0.5 * acc_imu * dt * dt;

  /* velocity of IMU */
  imu_prop_state.vel_end = imu_prop_state.vel_end + acc_imu * dt;
}

void LIVMapper::imu_prop_callback()
{
  if (p_imu->imu_need_init || !new_imu || !ekf_finish_once) { return; }
  mtx_buffer_imu_prop.lock();
  new_imu = false; // 控制 propagate 频率和 IMU 频率一致
  if (imu_prop_enable && !prop_imu_buffer.empty())
  {
    static double last_t_from_lidar_end_time = 0;
    if (state_update_flg)
    {
      imu_propagate = latest_ekf_state;
      // drop all useless imu pkg
      while ((!prop_imu_buffer.empty() && stamp2Sec(prop_imu_buffer.front().header.stamp) < latest_ekf_time))
      {
        prop_imu_buffer.pop_front();
      }
      last_t_from_lidar_end_time = 0;
      for (int i = 0; i < prop_imu_buffer.size(); i++)
      {
        double t_from_lidar_end_time = stamp2Sec(prop_imu_buffer[i].header.stamp) - latest_ekf_time;
        double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
        // cout << "prop dt" << dt << ", " << t_from_lidar_end_time << ", " << last_t_from_lidar_end_time << endl;
        V3D acc_imu(prop_imu_buffer[i].linear_acceleration.x, prop_imu_buffer[i].linear_acceleration.y, prop_imu_buffer[i].linear_acceleration.z);
        V3D omg_imu(prop_imu_buffer[i].angular_velocity.x, prop_imu_buffer[i].angular_velocity.y, prop_imu_buffer[i].angular_velocity.z);
        prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
        last_t_from_lidar_end_time = t_from_lidar_end_time;
      }
      state_update_flg = false;
    }
    else
    {
      V3D acc_imu(newest_imu.linear_acceleration.x, newest_imu.linear_acceleration.y, newest_imu.linear_acceleration.z);
      V3D omg_imu(newest_imu.angular_velocity.x, newest_imu.angular_velocity.y, newest_imu.angular_velocity.z);
      double t_from_lidar_end_time = stamp2Sec(newest_imu.header.stamp) - latest_ekf_time;
      double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
      prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
      last_t_from_lidar_end_time = t_from_lidar_end_time;
    }

    V3D posi, vel_i;
    Eigen::Quaterniond q;
    posi = imu_propagate.pos_end;
    vel_i = imu_propagate.vel_end;
    q = Eigen::Quaterniond(imu_propagate.rot_end);
    imu_prop_odom.header.frame_id = "world";
    imu_prop_odom.header.stamp = newest_imu.header.stamp;
    imu_prop_odom.pose.pose.position.x = posi.x();
    imu_prop_odom.pose.pose.position.y = posi.y();
    imu_prop_odom.pose.pose.position.z = posi.z();
    imu_prop_odom.pose.pose.orientation.w = q.w();
    imu_prop_odom.pose.pose.orientation.x = q.x();
    imu_prop_odom.pose.pose.orientation.y = q.y();
    imu_prop_odom.pose.pose.orientation.z = q.z();
    imu_prop_odom.twist.twist.linear.x = vel_i.x();
    imu_prop_odom.twist.twist.linear.y = vel_i.y();
    imu_prop_odom.twist.twist.linear.z = vel_i.z();
    pubImuPropOdom->publish(imu_prop_odom);
  }
  mtx_buffer_imu_prop.unlock();
}

void LIVMapper::transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud)
{
  PointCloudXYZI().swap(*trans_cloud);
  trans_cloud->reserve(input_cloud->size());
  for (size_t i = 0; i < input_cloud->size(); i++)
  {
    pcl::PointXYZINormal p_c = input_cloud->points[i];
    Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
    p = (rot * (extR * p + extT) + t);
    PointType pi;
    pi.x = p(0);
    pi.y = p(1);
    pi.z = p(2);
    pi.intensity = p_c.intensity;
    trans_cloud->points.push_back(pi);
  }
}

void LIVMapper::pointBodyToWorld(const PointType &pi, PointType &po)
{
  V3D p_body(pi.x, pi.y, pi.z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po.x = p_global(0);
  po.y = p_global(1);
  po.z = p_global(2);
  po.intensity = pi.intensity;
}

template <typename T> void LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
  V3D p_body(pi[0], pi[1], pi[2]);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po[0] = p_global(0);
  po[1] = p_global(1);
  po[2] = p_global(2);
}

template <typename T> Matrix<T, 3, 1> LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi)
{
  V3D p(pi[0], pi[1], pi[2]);
  p = (_state.rot_end * (extR * p + extT) + _state.pos_end);
  Eigen::Matrix<T, 3, 1> po(p[0], p[1], p[2]);
  return po;
}

void LIVMapper::RGBpointBodyToWorld(PointType const *const pi, PointType *const po)
{
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
  po->curvature = pi->curvature;
  po->normal_x = pi->normal_x;
  po->normal_y = pi->normal_y;
  po->normal_z = pi->normal_z;
}

void LIVMapper::RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po)
{
  V3D p_body_lidar(pi->x, pi->y, pi->z);
  V3D p_body_imu(extR * p_body_lidar + extT);

  po->x = p_body_imu(0);
  po->y = p_body_imu(1);
  po->z = p_body_imu(2);
  po->intensity = pi->intensity;
  po->curvature = pi->curvature;
  po->normal_x = pi->normal_x;
  po->normal_y = pi->normal_y;
  po->normal_z = pi->normal_z;
}

void LIVMapper::standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
{
  recordIngress(0, stampNanoseconds(msg->header.stamp));
  if (!lidar_en) return;
  double cur_head_time = stamp2Sec(msg->header.stamp) + lidar_time_offset;
  if (!std::isfinite(cur_head_time) || cur_head_time <= last_timestamp_lidar)
  {
    RCLCPP_ERROR(this->node->get_logger(), "Reject non-increasing/nonfinite LiDAR timestamp");
    return;
  }
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  try { p_pre->process(msg, ptr); }
  catch (const std::exception &error)
  {
    RCLCPP_ERROR(this->node->get_logger(), "Reject malformed LiDAR: %s", error.what());
    return;
  }
  if (!ptr || ptr->empty())
  {
    RCLCPP_ERROR(this->node->get_logger(), "Reject empty preprocessed LiDAR cloud");
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mtx_buffer);
    lid_raw_data_buffer.push_back(ptr);
    lid_header_time_buffer.push_back(cur_head_time);
    last_timestamp_lidar = cur_head_time;
  }
  sig_buffer.notify_all();
}

void LIVMapper::livox_pcl_cbk(const xq_livox_interfaces::msg::CustomMsg::ConstSharedPtr &msg_in)
{
  recordIngress(0, stampNanoseconds(msg_in->header.stamp));
  if (!lidar_en) return;
  mtx_buffer.lock();
  xq_livox_interfaces::msg::CustomMsg::SharedPtr msg(new xq_livox_interfaces::msg::CustomMsg(*msg_in));
  // if ((abs(stamp2Sec(msg->header.stamp) - last_timestamp_lidar) > 0.2 && last_timestamp_lidar > 0) || sync_jump_flag)
  // {
  //   ROS_WARN("lidar jumps %.3f\n", stamp2Sec(msg->header.stamp) - last_timestamp_lidar);
  //   sync_jump_flag = true;
  //   msg->header.stamp = rclcpp::Time().fromSec(last_timestamp_lidar + 0.1);
  // }
  if (abs(last_timestamp_imu - stamp2Sec(msg->header.stamp)) > 1.0 && !imu_buffer.empty())
  {
    double timediff_imu_wrt_lidar = last_timestamp_imu - stamp2Sec(msg->header.stamp);
    RCLCPP_INFO(this->node->get_logger(), "\033[95mSelf sync IMU and LiDAR, HARD time lag is %.10lf \n\033[0m", timediff_imu_wrt_lidar - 0.100);
    // imu_time_offset = timediff_imu_wrt_lidar;
  }

  double cur_head_time = stamp2Sec(msg->header.stamp);
  RCLCPP_INFO(this->node->get_logger(), "Get LiDAR, its header time: %.6f", cur_head_time);
  if (cur_head_time < last_timestamp_lidar)
  {
    RCLCPP_ERROR(this->node->get_logger(), "lidar loop back, clear buffer");
    lid_raw_data_buffer.clear();
    lid_header_time_buffer.clear();
  }
  RCLCPP_INFO(this->node->get_logger(), "get point cloud at time: %.6f", stamp2Sec(msg->header.stamp));
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);

  if (!ptr || ptr->empty()) {
    RCLCPP_ERROR(this->node->get_logger(), "Received an empty point cloud");
    mtx_buffer.unlock();
    return;
  }

  lid_raw_data_buffer.push_back(ptr);
  lid_header_time_buffer.push_back(cur_head_time);
  last_timestamp_lidar = cur_head_time;

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr &msg_in)
{
  recordIngress(1, stampNanoseconds(msg_in->header.stamp));
  if (!imu_en) return;
  // Cache initial IMU arrivals too. The LIVO synchronizer keeps only samples
  // after its first LiDAR epoch for initialization; receipt is not fusion.
  RCLCPP_INFO(this->node->get_logger(), "get imu at time: %.6f", stamp2Sec(msg_in->header.stamp));
  sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));
  msg->header.stamp = sec2Stamp(stamp2Sec(msg->header.stamp) - imu_time_offset);
  double timestamp = stamp2Sec(msg->header.stamp);

  if (last_timestamp_lidar >= 0.0 && fabs(last_timestamp_lidar - timestamp) > 0.5 && (!ros_driver_fix_en))
  {
    RCLCPP_WARN(this->node->get_logger(), "IMU and LiDAR not synced! delta time: %lf .\n", last_timestamp_lidar - timestamp);
  }

  if (ros_driver_fix_en && last_timestamp_lidar >= 0.0) timestamp += std::round(last_timestamp_lidar - timestamp);
  msg->header.stamp = sec2Stamp(timestamp);

  mtx_buffer.lock();

  if (last_timestamp_imu > 0.0 && timestamp < last_timestamp_imu)
  {
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    RCLCPP_ERROR(this->node->get_logger(), "imu loop back, offset: %lf \n", last_timestamp_imu - timestamp);
    return;
  }

  if (last_timestamp_imu > 0.0 && timestamp > last_timestamp_imu + 0.2)
  {
    RCLCPP_WARN(this->node->get_logger(), "imu time stamp Jumps %0.4lf seconds \n", timestamp - last_timestamp_imu);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    return;
  }

  last_timestamp_imu = timestamp;

  imu_buffer.push_back(msg);
  cout<<"got imu: "<<timestamp<<" imu size "<<imu_buffer.size()<<endl;
  mtx_buffer.unlock();
  if (imu_prop_enable)
  {
    mtx_buffer_imu_prop.lock();
    if (imu_prop_enable && !p_imu->imu_need_init) { prop_imu_buffer.push_back(*msg); }
    newest_imu = *msg;
    new_imu = true;
    mtx_buffer_imu_prop.unlock();
  }
  sig_buffer.notify_all();
}

cv::Mat LIVMapper::getImageFromMsg(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
{
  cv::Mat img;
  img = cv_bridge::toCvShare(img_msg, "bgr8")->image;
  return img;
}

// static int i = 0;
void LIVMapper::img_cbk(const sensor_msgs::msg::Image::ConstSharedPtr &msg_in)
{
  const int64_t original_stamp_ns = stampNanoseconds(msg_in->header.stamp);
  recordIngress(2, original_stamp_ns);
  const uint64_t image_sequence = ingress_counts[2];
  const auto discard = [&](const std::string &reason) {
    recordImageEpoch(image_sequence, original_stamp_ns, "startup_discard", reason, false, 0);
  };
  if (!img_en) { discard("image_input_disabled"); return; }
  sensor_msgs::msg::Image::SharedPtr msg(new sensor_msgs::msg::Image(*msg_in));
  // if ((abs(stamp2Sec(msg->header.stamp) - last_timestamp_img) > 0.2 && last_timestamp_img > 0) || sync_jump_flag)
  // {
  //   RCLCPP_WARN(this->node->get_logger(), "img jumps %.3f\n", stamp2Sec(msg->header.stamp) - last_timestamp_img);
  //   sync_jump_flag = true;
  //   msg->header.stamp = rclcpp::Time().fromSec(last_timestamp_img + 0.1);
  // }

  // Hiliti2022 40Hz
  if (hilti_en)
  {
    static int frame_counter = 0;
    if (++frame_counter % 4 != 0) { discard("hilti_decimation"); return; }
  }
  // double msg_header_time =  stamp2Sec(msg->header.stamp);
  double msg_header_time = stamp2Sec(msg->header.stamp) + img_time_offset;
  if (abs(msg_header_time - last_timestamp_img) < 0.001) { discard("duplicate_or_too_close_image_stamp"); return; }
  RCLCPP_INFO(this->node->get_logger(), "Get image, its header time: %.6f", msg_header_time);
  // DDS arrival order differs across sensor topics. Cache images even before
  // the first LiDAR callback; the synchronizer discards only image epochs at
  // or before the first LiDAR acquisition time once that reference is known.

  if (msg_header_time < last_timestamp_img)
  {
    RCLCPP_ERROR(this->node->get_logger(), "image loop back. \n");
    discard("non_increasing_image_stamp");
    return;
  }

  double img_time_correct = msg_header_time; // last_timestamp_lidar + 0.105;

  if (img_time_correct - last_timestamp_img < 0.02)
  {
    RCLCPP_WARN(this->node->get_logger(), "Image need Jumps: %.6f", img_time_correct);
    discard("image_interval_below_minimum");
    return;
  }

  cv::Mat img_cur;
  try { img_cur = getImageFromMsg(msg).clone(); }
  catch (const cv_bridge::Exception &error)
  {
    RCLCPP_ERROR(this->node->get_logger(), "Reject image conversion: %s", error.what());
    discard("image_conversion_failed");
    return;
  }
  if (img_cur.empty()) { discard("empty_image"); return; }
  {
    std::lock_guard<std::mutex> lock(mtx_buffer);
    img_buffer.push_back(img_cur);
    img_time_buffer.push_back(img_time_correct);
    img_header_time_buffer.push_back(stamp2Sec(msg_in->header.stamp));
    img_header_stamp_ns_buffer.push_back(original_stamp_ns);
    img_sequence_buffer.push_back(image_sequence);
    last_timestamp_img = img_time_correct;
  }

  // ROS_INFO("Correct Image time: %.6f", img_time_correct);

  // cv::imshow("img", img);
  // cv::waitKey(1);
  // cout<<"last_timestamp_img:::"<<last_timestamp_img<<endl;
  sig_buffer.notify_all();
}

void LIVMapper::jointDownCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
  ++joint_down_received;
  const int64_t stamp = stampNanoseconds(msg->header.stamp);
  if (stamp <= joint_last_down_ns || stamp <= 0)
    throw std::invalid_argument("Non-increasing joint downward acquisition stamp");
  joint_last_down_ns = stamp;
  bool valid = msg->width == 320 && msg->height == 240 && msg->encoding == "rgb8" &&
      msg->step == 960 && msg->data.size() == 230400 && msg->header.frame_id == "impact_down_optical";
  cv::Mat gray;
  if (valid)
  {
    try { gray = cv_bridge::toCvCopy(msg, "mono8")->image; }
    catch (const cv_bridge::Exception &) { valid = false; }
  }
  research_joint->input("down", stamp, valid, valid ? "raw_mono_from_rgb8" : "invalid_down_image_contract");
  if (valid) joint_down_images.emplace(stamp, gray);
  if (joint_down_images.size() > 10000) throw std::runtime_error("Joint downward buffer exceeded bound");
}

void LIVMapper::jointRangeCallback(const sensor_msgs::msg::Range::ConstSharedPtr &msg)
{
  ++joint_range_received;
  const int64_t stamp = stampNanoseconds(msg->header.stamp);
  if (stamp <= joint_last_range_ns || stamp <= 0)
    throw std::invalid_argument("Non-increasing joint range acquisition stamp");
  joint_last_range_ns = stamp;
  impact_research::EpochRange range;
  range.metres = msg->range; range.minimum = msg->min_range; range.maximum = msg->max_range;
  range.frame = msg->header.frame_id;
  range.available = std::isfinite(range.metres) && range.metres >= range.minimum && range.metres <= range.maximum &&
      range.minimum > 0. && range.maximum > range.minimum && range.frame == "impact_down_range" &&
      msg->radiation_type == sensor_msgs::msg::Range::INFRARED;
  research_joint->input("range", stamp, range.available, range.available ? "noisy_range_observation" : "invalid_range_contract");
  joint_ranges.emplace(stamp, range);
  if (joint_ranges.size() > 10000) throw std::runtime_error("Joint range buffer exceeded bound");
}

void LIVMapper::jointBarrierCallback(const std_msgs::msg::UInt64MultiArray::ConstSharedPtr &msg)
{
  if (msg->data.size() != 6 || msg->data[0] > static_cast<uint64_t>(std::numeric_limits<int64_t>::max()))
    throw std::invalid_argument("Invalid joint acquisition barrier");
  const int64_t stamp = static_cast<int64_t>(msg->data[0]);
  if (stamp <= 0 || stamp < joint_last_barrier_ns) throw std::invalid_argument("Regressing joint barrier");
  std::array<uint64_t,5> counts;
  std::copy_n(msg->data.begin() + 1, 5, counts.begin());
  const auto existing = joint_barriers.find(stamp);
  if (existing != joint_barriers.end() && existing->second != counts)
    throw std::invalid_argument("Conflicting duplicate joint barrier");
  for (size_t i = 0; i < counts.size(); ++i)
    if (counts[i] < joint_last_barrier_counts[i]) throw std::invalid_argument("Regressing joint barrier count");
  joint_last_barrier_ns = stamp;
  joint_last_barrier_counts = counts;
  joint_barriers.emplace(stamp, counts);
}

void LIVMapper::processJointEpoch(int64_t stamp_ns)
{
  cv::Mat down;
  impact_research::EpochRange range;
  const auto image = joint_down_images.find(stamp_ns);
  if (image != joint_down_images.end()) down = image->second;
  const auto measurement = joint_ranges.find(stamp_ns);
  if (measurement != joint_ranges.end()) range = measurement->second;
  research_joint->epoch(stamp_ns, _state, down, range);
  joint_down_images.erase(joint_down_images.begin(), joint_down_images.upper_bound(stamp_ns));
  joint_ranges.erase(joint_ranges.begin(), joint_ranges.upper_bound(stamp_ns));
  joint_barriers.erase(joint_barriers.begin(), joint_barriers.upper_bound(stamp_ns));
}

bool LIVMapper::sync_packages(LidarMeasureGroup &meas)
{
  if (research_joint_inputs && !img_header_stamp_ns_buffer.empty())
  {
    const auto barrier = joint_barriers.find(img_header_stamp_ns_buffer.front());
    if (barrier == joint_barriers.end()) return false;
    const std::array<uint64_t,5> received = {ingress_counts[1], ingress_counts[0], ingress_counts[2], joint_down_received, joint_range_received};
    for (size_t i = 0; i < received.size(); ++i)
      if (received[i] < barrier->second[i]) return false;
  }
  // A scheduled VIO epoch already has its image and propagated state; it does
  // not need another raw LiDAR/IMU sample to finish the LIO -> VIO transition.
  const bool pending_vio = slam_mode_ == LIVO && meas.lio_vio_flg == LIO;
  if (lid_raw_data_buffer.empty() && lidar_en && !pending_vio) return false;
  if (img_buffer.empty() && img_en) return false;
  if (imu_buffer.empty() && imu_en && !pending_vio) return false;

  switch (slam_mode_)
  {
  case ONLY_LIO:
  {
    if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();
    if (!lidar_pushed)
    {
      // If not push the lidar into measurement data buffer
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      if (meas.lidar->points.size() <= 1) return false;

      meas.lidar_frame_beg_time = lid_header_time_buffer.front();                                                // generate lidar_frame_beg_time
      meas.lidar_frame_end_time = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
      meas.pcl_proc_cur = meas.lidar;
      lidar_pushed = true;                                                                                       // flag
    }

    if (imu_en && last_timestamp_imu < meas.lidar_frame_end_time)
    { // waiting imu message needs to be
      // larger than _lidar_frame_end_time,
      // make sure complete propagate.
      // ROS_ERROR("out sync");
      return false;
    }

    struct MeasureGroup m; // standard method to keep imu message.

    m.imu.clear();
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    while (!imu_buffer.empty())
    {
      if (stamp2Sec(imu_buffer.front()->header.stamp) > meas.lidar_frame_end_time) break;
      m.imu.push_back(imu_buffer.front());
      imu_buffer.pop_front();
    }
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();

    meas.lio_vio_flg = LIO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    // ROS_INFO("ONlY HAS LiDAR and IMU, NO IMAGE!");
    lidar_pushed = false; // sync one whole lidar scan.
    return true;

    break;
  }

  case LIVO:
  {
    /*** For LIVO mode, the time of LIO update is set to be the same as VIO, LIO
     * first than VIO imediatly ***/
    EKF_STATE last_lio_vio_flg = meas.lio_vio_flg;
    // double t0 = omp_get_wtime();
    switch (last_lio_vio_flg)
    {
    // double img_capture_time = meas.lidar_frame_beg_time + exposure_time_init;
    case WAIT:
    case VIO:
    {
      // printf("!!! meas.lio_vio_flg: %d \n", meas.lio_vio_flg);
      double img_capture_time = img_time_buffer.front() + exposure_time_init;
      /*** has img topic, but img topic timestamp larger than lidar end time,
       * process lidar topic. After LIO update, the meas.lidar_frame_end_time
       * will be refresh. ***/
      if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();
      // printf("[ Data Cut ] wait \n");
      // printf("[ Data Cut ] last_lio_update_time: %lf \n",
      // meas.last_lio_update_time);

      double lid_newest_time = lid_header_time_buffer.back() + lid_raw_data_buffer.back()->points.back().curvature / double(1000);
      double imu_newest_time = stamp2Sec(imu_buffer.back()->header.stamp);

      if (img_capture_time < meas.last_lio_update_time + 0.00001)
      {
        recordImageEpoch(img_sequence_buffer.front(), img_header_stamp_ns_buffer.front(), "startup_discard", "not_after_current_lidar_epoch", false, 0);
        img_buffer.pop_front();
        img_time_buffer.pop_front();
        img_header_time_buffer.pop_front();
        img_header_stamp_ns_buffer.pop_front();
        img_sequence_buffer.pop_front();
        RCLCPP_ERROR(this->node->get_logger(), "[ Data Cut ] Throw one image frame! \n");
        return false;
      }

      if (img_capture_time > lid_newest_time || img_capture_time > imu_newest_time)
      {
        // RCLCPP_ERROR(this->node->get_logger(), "lost first camera frame");
        // printf("img_capture_time, lid_newest_time, imu_newest_time: %lf , %lf
        // , %lf \n", img_capture_time, lid_newest_time, imu_newest_time);
        return false;
      }

      struct MeasureGroup m;

      // printf("[ Data Cut ] LIO \n");
      // printf("[ Data Cut ] img_capture_time: %lf \n", img_capture_time);
      m.imu.clear();
      m.lio_time = img_capture_time;
      mtx_buffer.lock();
      while (!imu_buffer.empty())
      {
        if (stamp2Sec(imu_buffer.front()->header.stamp) > m.lio_time) break;

        if (stamp2Sec(imu_buffer.front()->header.stamp) > meas.last_lio_update_time) m.imu.push_back(imu_buffer.front());

        imu_buffer.pop_front();
        // printf("[ Data Cut ] imu time: %lf \n",
        // stamp2Sec(imu_buffer.front()->header.stamp));
      }
      mtx_buffer.unlock();
      sig_buffer.notify_all();

      *(meas.pcl_proc_cur) = *(meas.pcl_proc_next);
      PointCloudXYZI().swap(*meas.pcl_proc_next);

      int lid_frame_num = lid_raw_data_buffer.size();
      int max_size = meas.pcl_proc_cur->size() + 24000 * lid_frame_num;
      meas.pcl_proc_cur->reserve(max_size);
      meas.pcl_proc_next->reserve(max_size);
      // deque<PointCloudXYZI::Ptr> lidar_buffer_tmp;

      while (!lid_raw_data_buffer.empty())
      {
        if (lid_header_time_buffer.front() > img_capture_time) break;
        auto pcl(lid_raw_data_buffer.front()->points);
        double frame_header_time(lid_header_time_buffer.front());
        float max_offs_time_ms = (m.lio_time - frame_header_time) * 1000.0f;

        for (int i = 0; i < pcl.size(); i++)
        {
          auto pt = pcl[i];
          if (pcl[i].curvature < max_offs_time_ms ||
              (p_pre->lidar_type == GAZEBO_SNAPSHOT && pcl[i].curvature <= max_offs_time_ms))
          {
            pt.curvature += (frame_header_time - meas.last_lio_update_time) * 1000.0f;
            meas.pcl_proc_cur->points.push_back(pt);
          }
          else
          {
            pt.curvature += (frame_header_time - m.lio_time) * 1000.0f;
            meas.pcl_proc_next->points.push_back(pt);
          }
        }
        lid_raw_data_buffer.pop_front();
        lid_header_time_buffer.pop_front();
      }

      meas.measures.push_back(m);
      meas.lio_vio_flg = LIO;
      // meas.last_lio_update_time = m.lio_time;
      // printf("!!! meas.lio_vio_flg: %d \n", meas.lio_vio_flg);
      // printf("[ Data Cut ] pcl_proc_cur number: %d \n", meas.pcl_proc_cur
      // ->points.size()); printf("[ Data Cut ] LIO process time: %lf \n",
      // omp_get_wtime() - t0);
      return true;
    }

    case LIO:
    {
      double img_capture_time = img_time_buffer.front() + exposure_time_init;
      meas.lio_vio_flg = VIO;
      // printf("[ Data Cut ] VIO \n");
      meas.measures.clear();

      struct MeasureGroup m;
      m.vio_time = img_capture_time;
      m.image_header_time = img_header_time_buffer.front();
      m.image_header_stamp_ns = img_header_stamp_ns_buffer.front();
      m.image_sequence_id = img_sequence_buffer.front();
      m.lio_time = meas.last_lio_update_time;
      m.img = img_buffer.front();
      mtx_buffer.lock();
      // while ((!imu_buffer.empty() && (imu_time < img_capture_time)))
      // {
      //   imu_time = stamp2Sec(imu_buffer.front()->header.stamp);
      //   if (imu_time > img_capture_time) break;
      //   m.imu.push_back(imu_buffer.front());
      //   imu_buffer.pop_front();
      //   printf("[ Data Cut ] imu time: %lf \n",
      //   stamp2Sec(imu_buffer.front()->header.stamp));
      // }
      img_buffer.pop_front();
      img_time_buffer.pop_front();
      img_header_time_buffer.pop_front();
      img_header_stamp_ns_buffer.pop_front();
      img_sequence_buffer.pop_front();
      mtx_buffer.unlock();
      sig_buffer.notify_all();
      meas.measures.push_back(m);
      lidar_pushed = false; // after VIO update, the _lidar_frame_end_time will be refresh.
      // printf("[ Data Cut ] VIO process time: %lf \n", omp_get_wtime() - t0);
      return true;
    }

    default:
    {
      // printf("!! WRONG EKF STATE !!");
      return false;
    }
      // return false;
    }
    break;
  }

  case ONLY_LO:
  {
    if (!lidar_pushed) 
    { 
      // If not in lidar scan, need to generate new meas
      if (lid_raw_data_buffer.empty())  return false;
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      meas.lidar_frame_beg_time = lid_header_time_buffer.front(); // generate lidar_beg_time
      meas.lidar_frame_end_time  = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
      lidar_pushed = true;             
    }
    struct MeasureGroup m; // standard method to keep imu message.
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    lidar_pushed = false; // sync one whole lidar scan.
    meas.lio_vio_flg = LO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    return true;
    break;
  }

  default:
  {
    printf("!! WRONG SLAM TYPE !!");
    return false;
  }
  }
  RCLCPP_ERROR(this->node->get_logger(), "out sync");
  return false;
}

void LIVMapper::publish_img_rgb(const image_transport::Publisher &pubImage, VIOManagerPtr vio_manager)
{
  cv::Mat img_rgb = vio_manager->img_cp;
  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = sec2Stamp(LidarMeasures.last_lio_update_time);
  out_msg.header.frame_id = "impact_front_optical";
  out_msg.encoding = sensor_msgs::image_encodings::BGR8;
  out_msg.image = img_rgb;
  pubImage.publish(out_msg.toImageMsg());
}

// Provide output format for LiDAR-visual BA
void LIVMapper::publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFullRes, VIOManagerPtr vio_manager)
{
  if (pcl_w_wait_pub->empty()) return;
  PointCloudXYZRGB::Ptr laserCloudWorldRGB(new PointCloudXYZRGB());
  static int pub_num = 1;
  pub_num++;

  if (LidarMeasures.lio_vio_flg == VIO)
  {
    *pcl_wait_pub += *pcl_w_wait_pub;
    if(pub_num >= pub_scan_num)
    {
      pub_num = 1;
      size_t size = pcl_wait_pub->points.size();
      laserCloudWorldRGB->reserve(size);
      // double inv_expo = _state.inv_expo_time;
      cv::Mat img_rgb = vio_manager->img_rgb;
      for (size_t i = 0; i < size; i++)
      {
        PointTypeRGB pointRGB;
        pointRGB.x = pcl_wait_pub->points[i].x;
        pointRGB.y = pcl_wait_pub->points[i].y;
        pointRGB.z = pcl_wait_pub->points[i].z;

        V3D p_w(pcl_wait_pub->points[i].x, pcl_wait_pub->points[i].y, pcl_wait_pub->points[i].z);
        V3D pf(vio_manager->new_frame_->w2f(p_w)); if (pf[2] < 0) continue;
        V2D pc(vio_manager->new_frame_->w2c(p_w));

        if (vio_manager->new_frame_->cam_->isInFrame(pc.cast<int>(), 3)) // 100
        {
          V3F pixel = vio_manager->getInterpolatedPixel(img_rgb, pc);
          pointRGB.r = pixel[2];
          pointRGB.g = pixel[1];
          pointRGB.b = pixel[0];
          // pointRGB.r = pixel[2] * inv_expo; pointRGB.g = pixel[1] * inv_expo; pointRGB.b = pixel[0] * inv_expo;
          // if (pointRGB.r > 255) pointRGB.r = 255; else if (pointRGB.r < 0) pointRGB.r = 0;
          // if (pointRGB.g > 255) pointRGB.g = 255; else if (pointRGB.g < 0) pointRGB.g = 0;
          // if (pointRGB.b > 255) pointRGB.b = 255; else if (pointRGB.b < 0) pointRGB.b = 0;
          if (pf.norm() > blind_rgb_points) laserCloudWorldRGB->push_back(pointRGB);
        }
      }
    }
  }

  /*** Publish Frame ***/
  sensor_msgs::msg::PointCloud2 laserCloudmsg;
  if (slam_mode_ == LIVO && LidarMeasures.lio_vio_flg == VIO)
  {
    pcl::toROSMsg(*laserCloudWorldRGB, laserCloudmsg);
  }
  if (slam_mode_ == ONLY_LIO || slam_mode_ == ONLY_LO)
  { 
    pcl::toROSMsg(*pcl_w_wait_pub, laserCloudmsg); 
  }
  laserCloudmsg.header.stamp = sec2Stamp(LidarMeasures.last_lio_update_time);
  laserCloudmsg.header.frame_id = research_map_frame;
  pubLaserCloudFullRes->publish(laserCloudmsg);

  /**************** save map ****************/
  /* 1. make sure you have enough memories
  /* 2. noted that pcd save will influence the real-time performences **/
  double update_time = 0.0;
  if (LidarMeasures.lio_vio_flg == VIO) {
    update_time = LidarMeasures.measures.back().vio_time;
  } else { // LIO / LO
    update_time = LidarMeasures.measures.back().lio_time;
  }
  std::stringstream ss_time;
  ss_time << std::fixed << std::setprecision(6) << update_time;

  if (pcd_save_en)
  {
    static int scan_wait_num = 0;

    switch (pcd_save_type)
    {
      case 0: /** world frame **/
        if (slam_mode_ == LIVO)
        {
          *pcl_wait_save += *laserCloudWorldRGB;
        }
        else
        {
          *pcl_wait_save_intensity += *pcl_w_wait_pub;
        }
        if(LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO) scan_wait_num++;
        break;

      case 1: /** body frame **/
        if (LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO)
        {
          int size = feats_undistort->points.size();
          PointCloudXYZI::Ptr laserCloudBody(new PointCloudXYZI(size, 1));
          for (int i = 0; i < size; i++)
          {
            RGBpointBodyLidarToIMU(&feats_undistort->points[i], &laserCloudBody->points[i]);
          }
          *pcl_wait_save_intensity += *laserCloudBody;
          scan_wait_num++;
          cout << "save body frame points: " << pcl_wait_save_intensity->points.size() << endl;
        }
        pcd_save_interval = 1;
        
        break;

      default:
        pcd_save_interval = 1;
        scan_wait_num++;
        break;
    }
    if ((pcl_wait_save->size() > 0 || pcl_wait_save_intensity->size() > 0) && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval)
    {
      string all_points_dir(fast_livo_debug_file("pcd/" + ss_time.str() + ".pcd"));

      pcl::PCDWriter pcd_writer;

      cout << "current scan saved to " << all_points_dir << endl;
      if (pcl_wait_save->points.size() > 0)
      {
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save); // pcl::io::savePCDFileASCII(all_points_dir, *pcl_wait_save);
        PointCloudXYZRGB().swap(*pcl_wait_save);
      }
      if(pcl_wait_save_intensity->points.size() > 0)
      {
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_intensity);
        PointCloudXYZI().swap(*pcl_wait_save_intensity);
      }
      scan_wait_num = 0;
    }
    
    if(LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO)
    {
      Eigen::Quaterniond q(_state.rot_end);
      fout_lidar_pos << std::fixed << std::setprecision(6);
      fout_lidar_pos <<  LidarMeasures.measures.back().lio_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " " << q.x() << " " << q.y() << " " << q.z()
          << " " << q.w() << " " << endl;
    }
  }
  if (img_save_en && LidarMeasures.lio_vio_flg == VIO)
  {
    static int img_wait_num = 0;
    img_wait_num++;

    if (img_save_interval > 0 && img_wait_num >= img_save_interval)
    {
      imwrite(fast_livo_debug_file("image/" + ss_time.str() + ".png"), vio_manager->img_rgb);
      
      Eigen::Quaterniond q(_state.rot_end);
      fout_visual_pos << std::fixed << std::setprecision(6);
      fout_visual_pos << LidarMeasures.measures.back().vio_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
      img_wait_num = 0;
    }
  }

  if(laserCloudWorldRGB->size() > 0)  PointCloudXYZI().swap(*pcl_wait_pub); 
  if(LidarMeasures.lio_vio_flg == VIO)  PointCloudXYZI().swap(*pcl_w_wait_pub);
}

void LIVMapper::publish_visual_sub_map(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubSubVisualMap)
{
  PointCloudXYZI::Ptr laserCloudFullRes(visual_sub_map);
  int size = laserCloudFullRes->points.size(); if (size == 0) return;
  PointCloudXYZI::Ptr sub_pcl_visual_map_pub(new PointCloudXYZI());
  *sub_pcl_visual_map_pub = *laserCloudFullRes;
  if (1)
  {
    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*sub_pcl_visual_map_pub, laserCloudmsg);
    laserCloudmsg.header.stamp = sec2Stamp(LidarMeasures.last_lio_update_time);
    laserCloudmsg.header.frame_id = research_map_frame;
    pubSubVisualMap->publish(laserCloudmsg);
  }
}

void LIVMapper::publish_effect_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list)
{
  int effect_feat_num = ptpl_list.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(effect_feat_num, 1));
  for (int i = 0; i < effect_feat_num; i++)
  {
    laserCloudWorld->points[i].x = ptpl_list[i].point_w_[0];
    laserCloudWorld->points[i].y = ptpl_list[i].point_w_[1];
    laserCloudWorld->points[i].z = ptpl_list[i].point_w_[2];
  }
  sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
  pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
  laserCloudFullRes3.header.stamp = sec2Stamp(LidarMeasures.last_lio_update_time);
  laserCloudFullRes3.header.frame_id = research_map_frame;
  pubLaserCloudEffect->publish(laserCloudFullRes3);
}

template <typename T> void LIVMapper::set_posestamp(T &out)
{
  out.position.x = _state.pos_end(0);
  out.position.y = _state.pos_end(1);
  out.position.z = _state.pos_end(2);
  out.orientation.x = geoQuat.x;
  out.orientation.y = geoQuat.y;
  out.orientation.z = geoQuat.z;
  out.orientation.w = geoQuat.w;
}

void LIVMapper::recordIngress(size_t sensor, int64_t stamp_ns)
{
  if (ingress_counts[sensor]++ == 0)
  {
    ingress_first_stamp[sensor] = static_cast<double>(stamp_ns) * 1e-9;
    ingress_first_stamp_ns[sensor] = stamp_ns;
  }
  ingress_last_stamp[sensor] = static_cast<double>(stamp_ns) * 1e-9;
  ingress_last_stamp_ns[sensor] = stamp_ns;
}

void LIVMapper::recordImageEpoch(uint64_t sequence, int64_t stamp_ns, const std::string &classification,
                                 const std::string &reason, bool odometry_published, uint64_t lidar_points)
{
  if (!recorded_image_sequences.insert(sequence).second)
    throw std::runtime_error("An image message was accounted more than once");
  size_t classification_index = 4;
  for (size_t index = 0; index < 4; ++index)
    if (classification == imageClassifications[index]) classification_index = index;
  if (classification_index == 4 || !research_image_ledger.is_open())
    throw std::runtime_error("Image processing ledger is unavailable or classification is invalid");
  ++image_classification_counts[classification_index];
  recorded_image_classifications.emplace_back(stamp_ns, classification_index);
  if (classification_index == 0) last_completed_post_vio_header_stamp_ns = stamp_ns;
  auto &out = research_image_ledger;
  out << "{\"schema_version\":1,\"event\":\"image_epoch\",\"image_sequence_id\":" << sequence
      << ",\"header_stamp_ns\":" << stamp_ns << ",\"classification\":" << jsonString(classification)
      << ",\"epoch_index\":";
  if (classification_index == 0) out << research_epoch_index;
  else out << "null";
  out << ",\"lio_points\":" << lidar_points << ",\"odom_published\":" << (odometry_published ? "true" : "false")
      << ",\"reason\":" << jsonString(reason) << "}\n";
  out.flush();
  if (!out) throw std::runtime_error("Image processing ledger write failed");
}

void LIVMapper::tryFinalizeProcessing()
{
  if (processing_result_written) return;
  for (size_t sensor = 0; sensor < 3; ++sensor)
    if (ingress_counts[sensor] != static_cast<uint64_t>(expected_messages[sensor])) return;
  if (ingress_last_stamp_ns[0] != expected_last_lidar_stamp_ns || ingress_last_stamp_ns[1] != expected_last_imu_stamp_ns) return;
  if (research_joint_inputs)
  {
    const std::array<uint64_t,5> expected = {static_cast<uint64_t>(expected_messages[1]), static_cast<uint64_t>(expected_messages[0]),
        static_cast<uint64_t>(expected_messages[2]), static_cast<uint64_t>(expected_joint_down), static_cast<uint64_t>(expected_joint_range)};
    if (joint_down_received != static_cast<uint64_t>(expected_joint_down) || joint_range_received != static_cast<uint64_t>(expected_joint_range) ||
        joint_last_down_ns != expected_joint_last_down_ns || joint_last_range_ns != expected_joint_last_range_ns ||
        joint_last_barrier_ns != expected_joint_last_barrier_ns || joint_last_barrier_counts != expected) return;
  }

  // Only after every frozen input has arrived can an image be conclusively
  // classified as beyond all available LiDAR/IMU support. Never relabel an
  // unprocessed eligible image as terminal just to finish.
  const int64_t coverage_end = std::min(expected_last_lidar_stamp_ns, expected_last_imu_stamp_ns);
  while (!img_buffer.empty() && img_header_stamp_ns_buffer.front() > coverage_end)
  {
    const int64_t stamp_ns = img_header_stamp_ns_buffer.front();
    recordImageEpoch(img_sequence_buffer.front(), stamp_ns, "terminal_uncovered",
                     stamp_ns > expected_last_lidar_stamp_ns ? "after_last_lidar_header" : "after_last_imu_header", false, 0);
    img_buffer.pop_front();
    img_time_buffer.pop_front();
    img_header_time_buffer.pop_front();
    img_header_stamp_ns_buffer.pop_front();
    img_sequence_buffer.pop_front();
  }
  if (recorded_image_sequences.size() != static_cast<uint64_t>(expected_messages[2]) ||
      last_completed_post_vio_header_stamp_ns != expected_last_eligible_image_stamp_ns ||
      visual_diagnostic_rows != image_classification_counts[0] || odometry_rows != image_classification_counts[0]) return;

  uint64_t eligible_rows = 0;
  bool found_last_eligible = false;
  for (const auto &entry : recorded_image_classifications)
  {
    const bool eligible = entry.first > ingress_first_stamp_ns[0] && entry.first <= coverage_end;
    if (eligible)
    {
      ++eligible_rows;
      if (entry.second != 0 && entry.second != 2) return;
      if (entry.first > expected_last_eligible_image_stamp_ns) return;
    }
    else if (entry.first <= ingress_first_stamp_ns[0])
    {
      if (entry.second != 1) return;
    }
    else if (entry.second != 3) return;
    if (entry.first == expected_last_eligible_image_stamp_ns && entry.second == 0) found_last_eligible = true;
  }
  if (!found_last_eligible) return;
  research_visual_log.flush();
  research_odometry_log.flush();
  research_image_ledger.flush();
  if (!research_visual_log || !research_odometry_log || !research_image_ledger)
    throw std::runtime_error("Cannot flush research processing evidence");

  const auto result_file = std::filesystem::path(research_run_file).parent_path() / "processing-result.json";
  const auto temporary_file = std::filesystem::path(result_file.string() + ".tmp");
  std::ofstream out(temporary_file);
  out << "{\"schema_version\":1,\"status\":\"COMPLETE\",\"complete\":true,\"joint_inputs_enabled\":" << (research_joint_inputs ? "true" : "false");
  if (research_joint_inputs)
    out << ",\"joint_inputs\":{\"received_down\":" << joint_down_received << ",\"received_range\":" << joint_range_received
        << ",\"last_down_stamp_ns\":" << joint_last_down_ns << ",\"last_range_stamp_ns\":" << joint_last_range_ns
        << ",\"last_barrier_stamp_ns\":" << joint_last_barrier_ns << ",\"all_five_counts_and_tails_verified\":true}";
  out << ",\"expected_messages\":{";
  const char *names[] = {"lidar", "imu", "image"};
  for (size_t sensor = 0; sensor < 3; ++sensor)
  {
    if (sensor) out << ',';
    out << jsonString(names[sensor]) << ':' << expected_messages[sensor];
  }
  out << "},\"received_messages\":{";
  for (size_t sensor = 0; sensor < 3; ++sensor)
  {
    if (sensor) out << ',';
    out << jsonString(names[sensor]) << ':' << ingress_counts[sensor];
  }
  out << "},\"first_header_stamp_ns\":{";
  for (size_t sensor = 0; sensor < 3; ++sensor)
  {
    if (sensor) out << ',';
    out << jsonString(names[sensor]) << ':' << ingress_first_stamp_ns[sensor];
  }
  out << "},\"last_header_stamp_ns\":{";
  for (size_t sensor = 0; sensor < 3; ++sensor)
  {
    if (sensor) out << ',';
    out << jsonString(names[sensor]) << ':' << ingress_last_stamp_ns[sensor];
  }
  out << "},\"image_classification_counts\":{";
  for (size_t index = 0; index < 4; ++index)
  {
    if (index) out << ',';
    out << jsonString(imageClassifications[index]) << ':' << image_classification_counts[index];
  }
  out << "},\"last_eligible_image_stamp_ns\":" << expected_last_eligible_image_stamp_ns
      << ",\"last_completed_post_vio_header_stamp_ns\":" << last_completed_post_vio_header_stamp_ns
      << ",\"image_ledger_rows\":" << recorded_image_sequences.size()
      << ",\"visual_diagnostic_rows\":" << visual_diagnostic_rows << ",\"odometry_rows\":" << odometry_rows
      << ",\"eligible_image_rows\":" << eligible_rows
      << ",\"diagnostics_flushed\":true,\"odometry_flushed\":true,\"ledger_flushed\":true,"
      << "\"all_inputs_received\":true,\"all_image_messages_accounted\":true}\n";
  out.flush();
  if (!out) throw std::runtime_error("Cannot write processing completion evidence");
  out.close();
  if (!out) throw std::runtime_error("Cannot close processing completion evidence");
  std::filesystem::rename(temporary_file, result_file);
  processing_result_written = true;
}

void LIVMapper::writeIngressResult()
{
  if (ingress_result_written || research_run_file.empty()) return;
  const auto file = std::filesystem::path(research_run_file).parent_path() / "ingress-result.json";
  if (std::filesystem::exists(file)) throw std::runtime_error("Refuse to overwrite ingress-result.json");
  std::ofstream out(file);
  out << "{\"schema_version\":1,\"event\":\"ingress_result\",\"source_lidar_type\":" << p_pre->lidar_type
      << ",\"snapshot_zero_within_cloud_offsets\":" << (p_pre->lidar_type == GAZEBO_SNAPSHOT ? "true" : "false")
      << ",\"hardware_validated\":false,\"sensors\":{";
  const char *names[] = {"lidar", "imu", "image"};
  for (size_t sensor = 0; sensor < 3; ++sensor)
  {
    if (sensor) out << ',';
    out << jsonString(names[sensor]) << ":{\"received\":" << ingress_counts[sensor] << ",\"first_header_stamp_s\":";
    jsonNumber(out, ingress_first_stamp[sensor]);
    out << ",\"last_header_stamp_s\":";
    jsonNumber(out, ingress_last_stamp[sensor]);
    out << '}';
  }
  out << "}}\n";
  out.flush();
  if (!out) throw std::runtime_error("Cannot write ingress-result.json");
  ingress_result_written = true;
}

void LIVMapper::recordVisualUpdate(const StatesGroup &before, const StatesGroup *state_before, double image_stamp)
{
  ++research_epoch_index;
  if (!research_visual_log.is_open()) return;
  const bool finite = stateFinite(before) && stateFinite(_state);
  const bool same_object = state_before == &_state && vio_manager->state == &_state;
  const double delta_position = (_state.pos_end - before.pos_end).norm();
  // Diagnostic geodesic from the same normalized quaternions written to JSON.
  // The upstream algorithm's small-angle Log deliberately returns sin(theta)
  // in this range; keep that algorithm unchanged and report the actual angle.
  Eigen::Quaterniond before_quaternion(before.rot_end), after_quaternion(_state.rot_end);
  before_quaternion.normalize();
  after_quaternion.normalize();
  const Eigen::Quaterniond relative_quaternion = (after_quaternion * before_quaternion.conjugate()).normalized();
  const double delta_rotation = finite ? 2.0 * std::atan2(relative_quaternion.vec().norm(), std::abs(relative_quaternion.w())) : std::numeric_limits<double>::quiet_NaN();
  const double covariance_delta = (_state.cov - before.cov).norm();
  const bool corrected = research_visual_update_enabled && same_object && finite && vio_manager->total_points > 0 &&
      (delta_position > 1e-12 || delta_rotation > 1e-12) && covariance_delta > 1e-12;
  std::ostringstream address_before, address_after;
  address_before << static_cast<const void *>(state_before);
  address_after << static_cast<const void *>(vio_manager->state);
  auto &out = research_visual_log;
  out << "{\"schema_version\":1,\"event\":\"visual_update\",\"epoch_index\":" << research_epoch_index
      << ",\"first_post_vio_epoch\":" << (research_epoch_index == 1 ? "true" : "false") << ",\"stamp_s\":";
  jsonNumber(out, LidarMeasures.last_lio_update_time);
  out << ",\"image_stamp_s\":"; jsonNumber(out, image_stamp);
  out << ",\"visual_update_enabled\":" << (research_visual_update_enabled ? "true" : "false")
      << ",\"fresh_lio_points_at_this_epoch\":" << latest_lio_points
      << ",\"existing_map_without_new_lio_snapshot\":" << (latest_lio_points == 0 && lidar_map_inited ? "true" : "false")
      << ",\"last_nonempty_lio_state_epoch_s\":";
  jsonNumber(out, last_nonempty_lio_update_time);
  out << ",\"lio_map_update_age_s\":"; jsonNumber(out, LidarMeasures.last_lio_update_time - last_nonempty_lio_update_time);
  out
      << ",\"selected_points\":" << vio_manager->total_points
      << ",\"correction_applied\":" << (corrected ? "true" : "false")
      << ",\"same_state_object\":" << (same_object ? "true" : "false")
      << ",\"state_address_before\":" << jsonString(address_before.str())
      << ",\"state_address_after\":" << jsonString(address_after.str()) << ",\"pose_before\":";
  jsonPose(out, before);
  out << ",\"pose_after\":"; jsonPose(out, _state);
  out << ",\"covariance_before\":"; jsonMatrix(out, before.cov);
  out << ",\"covariance_after\":"; jsonMatrix(out, _state.cov);
  out << ",\"delta_position_m\":"; jsonNumber(out, delta_position);
  out << ",\"delta_rotation_rad\":"; jsonNumber(out, delta_rotation);
  out << ",\"covariance_delta_frobenius\":"; jsonNumber(out, covariance_delta);
  out << ",\"imu_initialized\":" << (!p_imu->imu_need_init ? "true" : "false")
      << ",\"accepted\":" << (finite ? "true" : "false")
      << ",\"provenance\":{\"shared_state_id\":\"fast_livo2_19d\",\"visual_uses_lidar_map\":true,\"imu_state_propagation_shared\":true}"
      << ",\"hardware_validated\":false}\n";
  out.flush();
  if (!out) throw std::runtime_error("Visual diagnostics write failed");
  ++visual_diagnostic_rows;
}

void LIVMapper::publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &pubOdomAftMapped)
{
  const auto covariance = poseCovariance(_state);
  const Eigen::Quaterniond quaternion(_state.rot_end);
  geoQuat.x = quaternion.x(); geoQuat.y = quaternion.y(); geoQuat.z = quaternion.z(); geoQuat.w = quaternion.w();
  odomAftMapped = nav_msgs::msg::Odometry();
  odomAftMapped.header.frame_id = research_map_frame;
  odomAftMapped.child_frame_id = research_body_frame;
  odomAftMapped.header.stamp = sec2Stamp(LidarMeasures.last_lio_update_time);
  set_posestamp(odomAftMapped.pose.pose);
  for (size_t row = 0; row < 6; ++row)
    for (size_t column = 0; column < 6; ++column)
      odomAftMapped.pose.covariance[6 * row + column] = covariance(row, column);
  // nav_msgs twist is in the child frame; angular velocity/covariance are not
  // estimated here. NaN explicitly avoids advertising default-zero certainty.
  const V3D body_velocity = _state.rot_end.transpose() * _state.vel_end;
  odomAftMapped.twist.twist.linear.x = body_velocity.x();
  odomAftMapped.twist.twist.linear.y = body_velocity.y();
  odomAftMapped.twist.twist.linear.z = body_velocity.z();
  const double unavailable = std::numeric_limits<double>::quiet_NaN();
  odomAftMapped.twist.twist.angular.x = unavailable;
  odomAftMapped.twist.twist.angular.y = unavailable;
  odomAftMapped.twist.twist.angular.z = unavailable;
  odomAftMapped.twist.covariance.fill(unavailable);
  pubOdomAftMapped->publish(odomAftMapped);
  if (research_odometry_log.is_open())
  {
    auto &out = research_odometry_log;
    out << "{\"schema_version\":1,\"event\":\"odometry\",\"epoch_index\":" << research_epoch_index << ",\"stamp_s\":";
    jsonNumber(out, LidarMeasures.last_lio_update_time);
    out << ",\"frame_id\":" << jsonString(research_map_frame) << ",\"child_frame_id\":" << jsonString(research_body_frame)
        << ",\"position\":"; jsonVector(out, _state.pos_end);
    out << ",\"quaternion_xyzw\":"; jsonVector(out, quaternion.coeffs());
    out << ",\"velocity_world_mps\":"; jsonVector(out, _state.vel_end);
    out << ",\"covariance\":"; jsonMatrix(out, _state.cov);
    out << ",\"pose_covariance\":"; jsonMatrix(out, covariance);
    out << ",\"pose_covariance_basis\":\"position_world_rotation_world_tangent\""
        << ",\"visual_update_enabled\":" << (research_visual_update_enabled ? "true" : "false")
        << ",\"post_vio_epoch\":" << (slam_mode_ == LIVO ? "true" : "false")
        << ",\"twist_angular_available\":false,\"twist_covariance_available\":false,\"hardware_validated\":false}\n";
    out.flush();
    if (!out) throw std::runtime_error("Odometry diagnostics write failed");
    ++odometry_rows;
  }
}

void LIVMapper::publish_mavros(const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr &mavros_pose_publisher)
{
  (void)mavros_pose_publisher; // Intentionally unavailable in this shadow build.
}

void LIVMapper::publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &pubPath)
{
  set_posestamp(msg_body_pose.pose);
  msg_body_pose.header.stamp = sec2Stamp(LidarMeasures.last_lio_update_time);
  msg_body_pose.header.frame_id = research_map_frame;
  path.poses.push_back(msg_body_pose);
  pubPath->publish(path);
}
