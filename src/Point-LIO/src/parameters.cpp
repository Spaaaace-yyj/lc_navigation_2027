#include "parameters.h"

bool is_first_frame = true;
double lidar_end_time = 0.0, first_lidar_time = 0.0, time_con = 0.0;
double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0;
int pcd_index = 0;
IVoxType::Options ivox_options_;
int ivox_nearby_type = 6;

std::vector<double> extrinT(3, 0.0);
std::vector<double> extrinR(9, 0.0);
state_input state_in;
state_output state_out;
std::string lid_topic, imu_topic;
bool prop_at_freq_of_imu = true, check_satu = true, con_frame = false, cut_frame = false;
bool use_imu_as_input = false, space_down_sample = true, publish_odometry_without_downsample = false;
int  init_map_size = 10, con_frame_num = 1;
double match_s = 81, satu_acc, satu_gyro, cut_frame_time_interval = 0.1;
float  plane_thr = 0.1f;
double filter_size_surf_min = 0.5, filter_size_map_min = 0.5, fov_deg = 180;
// double cube_len = 2000; 
float  DET_RANGE = 450;
bool   imu_en = true;
double imu_time_inte = 0.005;
double laser_point_cov = 0.01, acc_norm;
double vel_cov, acc_cov_input, gyr_cov_input;
double gyr_cov_output, acc_cov_output, b_gyr_cov, b_acc_cov;
double imu_meas_acc_cov, imu_meas_omg_cov; 
int    lidar_type, pcd_save_interval;
std::vector<double> gravity_init, gravity;
bool   runtime_pos_log, pcd_save_en, path_en, extrinsic_est_en = true;
bool   scan_pub_en, scan_body_pub_en;
shared_ptr<Preprocess> p_pre;
shared_ptr<ImuProcess> p_imu;
double time_update_last = 0.0, time_current = 0.0, time_predict_last_const = 0.0, t_last = 0.0;
double time_diff_lidar_to_imu = 0.0;

double lidar_time_inte = 0.1, first_imu_time = 0.0;
int cut_frame_num = 1, orig_odom_freq = 10;
double online_refine_time = 20.0; //unit: s
bool cut_frame_init = false; // true;

MeasureGroup Measures;

ofstream fout_out, fout_imu_pbp;

void readParameters(rclcpp::Node::SharedPtr nh)
{
  p_pre.reset(new Preprocess());
  p_imu.reset(new ImuProcess());

  prop_at_freq_of_imu = nh->declare_parameter<bool>("prop_at_freq_of_imu", true);
  use_imu_as_input    = nh->declare_parameter<bool>("use_imu_as_input", false);
  check_satu          = nh->declare_parameter<bool>("check_satu", true);
  init_map_size       = nh->declare_parameter<int>("init_map_size", 100);
  space_down_sample   = nh->declare_parameter<bool>("space_down_sample", true);

  satu_acc  = nh->declare_parameter<double>("mapping/satu_acc", 3.0);
  satu_gyro = nh->declare_parameter<double>("mapping/satu_gyro", 35.0);
  acc_norm  = nh->declare_parameter<double>("mapping/acc_norm", 1.0);
  plane_thr = static_cast<float>(nh->declare_parameter<double>("mapping/plane_thr", 0.05));

  p_pre->point_filter_num = nh->declare_parameter<int>("point_filter_num", 2);

  lid_topic = nh->declare_parameter<std::string>("common/lid_topic", "/livox/lidar");
  imu_topic = nh->declare_parameter<std::string>("common/imu_topic", "/livox/imu");

  con_frame               = nh->declare_parameter<bool>("common/con_frame", false);
  con_frame_num           = nh->declare_parameter<int>("common/con_frame_num", 1);
  cut_frame               = nh->declare_parameter<bool>("common/cut_frame", false);
  cut_frame_time_interval = nh->declare_parameter<double>("common/cut_frame_time_interval", 0.1);
  time_diff_lidar_to_imu  = nh->declare_parameter<double>("common/time_diff_lidar_to_imu", 0.0);

  filter_size_surf_min = nh->declare_parameter<double>("filter_size_surf", 0.5);
  filter_size_map_min  = nh->declare_parameter<double>("filter_size_map", 0.5);
  DET_RANGE            = static_cast<float>(nh->declare_parameter<double>("mapping/det_range", 300.0));
  fov_deg              = nh->declare_parameter<double>("mapping/fov_degree", 180.0);
  imu_en               = nh->declare_parameter<bool>("mapping/imu_en", true);
  extrinsic_est_en     = nh->declare_parameter<bool>("mapping/extrinsic_est_en", true);
  imu_time_inte        = nh->declare_parameter<double>("mapping/imu_time_inte", 0.005);
  laser_point_cov      = nh->declare_parameter<double>("mapping/lidar_meas_cov", 0.1);
  acc_cov_input        = nh->declare_parameter<double>("mapping/acc_cov_input", 0.1);
  vel_cov              = nh->declare_parameter<double>("mapping/vel_cov", 20.0);
  gyr_cov_input        = nh->declare_parameter<double>("mapping/gyr_cov_input", 0.1);
  gyr_cov_output       = nh->declare_parameter<double>("mapping/gyr_cov_output", 0.1);
  acc_cov_output       = nh->declare_parameter<double>("mapping/acc_cov_output", 0.1);
  b_gyr_cov            = nh->declare_parameter<double>("mapping/b_gyr_cov", 0.0001);
  b_acc_cov            = nh->declare_parameter<double>("mapping/b_acc_cov", 0.0001);
  imu_meas_acc_cov     = nh->declare_parameter<double>("mapping/imu_meas_acc_cov", 0.1);
  imu_meas_omg_cov     = nh->declare_parameter<double>("mapping/imu_meas_omg_cov", 0.1);

  p_pre->blind       = nh->declare_parameter<double>("preprocess/blind", 1.0);
  lidar_type         = nh->declare_parameter<int>("preprocess/lidar_type", 1);
  p_pre->N_SCANS     = nh->declare_parameter<int>("preprocess/scan_line", 16);
  p_pre->SCAN_RATE   = nh->declare_parameter<int>("preprocess/scan_rate", 10);
  p_pre->time_unit   = nh->declare_parameter<int>("preprocess/timestamp_unit", 1);

  match_s = nh->declare_parameter<double>("mapping/match_s", 81.0);

  gravity      = nh->declare_parameter<std::vector<double>>("mapping/gravity", std::vector<double>());
  gravity_init = nh->declare_parameter<std::vector<double>>("mapping/gravity_init", std::vector<double>());
  extrinT      = nh->declare_parameter<std::vector<double>>("mapping/extrinsic_T", std::vector<double>());
  extrinR      = nh->declare_parameter<std::vector<double>>("mapping/extrinsic_R", std::vector<double>());

  publish_odometry_without_downsample =
      nh->declare_parameter<bool>("odometry/publish_odometry_without_downsample", false);

  path_en         = nh->declare_parameter<bool>("publish/path_en", true);
  scan_pub_en     = nh->declare_parameter<bool>("publish/scan_publish_en", true);
  scan_body_pub_en = nh->declare_parameter<bool>("publish/scan_bodyframe_pub_en", true);
  runtime_pos_log = nh->declare_parameter<bool>("runtime_pos_log_enable", false);

  pcd_save_en     = nh->declare_parameter<bool>("pcd_save/pcd_save_en", false);
  pcd_save_interval = nh->declare_parameter<int>("pcd_save/interval", -1);

  lidar_time_inte  = nh->declare_parameter<double>("mapping/lidar_time_inte", 0.1);
  ivox_options_.resolution_ =
      static_cast<float>(nh->declare_parameter<double>("mapping/ivox_grid_resolution", 0.2));
  ivox_nearby_type = nh->declare_parameter<int>("ivox_nearby_type", 18);

  if (ivox_nearby_type == 0) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
  } else if (ivox_nearby_type == 6) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
  } else if (ivox_nearby_type == 18) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
  } else if (ivox_nearby_type == 26) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
  } else {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
  }

  p_imu->gravity_ << VEC_FROM_ARRAY(gravity);
}

Eigen::Matrix<double, 3, 1> SO3ToEuler(const SO3 &rot) 
{
    double sy = sqrt(rot(0,0)*rot(0,0) + rot(1,0)*rot(1,0));
    bool singular = sy < 1e-6;
    double x, y, z;
    if(!singular)
    {
        x = atan2(rot(2, 1), rot(2, 2));
        y = atan2(-rot(2, 0), sy);   
        z = atan2(rot(1, 0), rot(0, 0));  
    }
    else
    {    
        x = atan2(-rot(1, 2), rot(1, 1));    
        y = atan2(-rot(2, 0), sy);    
        z = 0;
    }
    Eigen::Matrix<double, 3, 1> ang(x, y, z);
    return ang;
}

void open_file()
{

    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
    fout_imu_pbp.open(DEBUG_FILE_DIR("imu_pbp.txt"),ios::out);
    if (fout_out && fout_imu_pbp)
        cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
    else
        cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;

}

void reset_cov(Eigen::Matrix<double, 24, 24> & P_init)
{
    P_init = MD(24, 24)::Identity() * 0.1;
    P_init.block<3, 3>(21, 21) = MD(3,3)::Identity() * 0.0001;
    P_init.block<6, 6>(15, 15) = MD(6,6)::Identity() * 0.001;
}

void reset_cov_output(Eigen::Matrix<double, 30, 30> & P_init_output)
{
    P_init_output = MD(30, 30)::Identity() * 0.01;
    P_init_output.block<3, 3>(21, 21) = MD(3,3)::Identity() * 0.0001;
    // P_init_output.block<6, 6>(6, 6) = MD(6,6)::Identity() * 0.0001;
    P_init_output.block<6, 6>(24, 24) = MD(6,6)::Identity() * 0.001;
}