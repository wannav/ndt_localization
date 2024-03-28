#ifndef _NDT_H_
#define _NDT_H_

#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <sstream>
#include <string>
#include <iomanip>
#include <deque>

#include <ros/ros.h>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>

#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>

#define MAX_MEASUREMENT_RANGE 120.0

enum class SensorType
{
  VELODYNE,
  OUSTER
};

class ParamServer
{
public:
  ros::NodeHandle nh;

  // Topics
  std::string pointCloudTopic;
  std::string gpsTopic;
  std::string mapTopic;

  // Frames
  std::string lidarFrame;
  std::string baseFrame;
  std::string odometryFrame;
  std::string mapFrame;
  std::string gpsFrame;

  // output
  std::string output_path;

  // frequence control
  float mappingProcessInterval;

  // NDT
  double trans_epsilon;
  double step_size;
  double resolution;
  int max_iterations;
  double converged_param_transform_probability;
  double vg_scan_size;
  double radius;

  // GPS
  double gpsCovThreshold;

  int ndtDelayTime; // unit(s)

  ParamServer();
};

class NdtLocalizer : public ParamServer
{
public:
  NdtLocalizer(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
  ~NdtLocalizer();

  enum Status
  {
    sys_none,
    sys_init_pose,
    sys_run
  };
  Status sys_status;

  void InitDelay(int delaySec);


private:
  bool usrDebug = false;

  // map loader
  bool getCurPose = true;
  ros::Publisher pubCurPose;             // 发布ndt配准得到的位姿
  geometry_msgs::PoseStamped curPoseMsg; // 用于加载地图的位姿

  // output pose
  std::string outputPath;
  std::ofstream ndtPoseFile;
  std::ofstream gpsOdomFile;

  // downsampler
  // double voxelLeafSize = 1.0;
  bool _output_log = false;
  std::ofstream ofs;
  std::string filename;

  std::string POINTS_TOPIC;
  double measurement_range = MAX_MEASUREMENT_RANGE;

  tf::TransformListener tfListener;
  tf::StampedTransform Baselink2GPS;

  // ndt
  bool initPose = false;    //是否已经得到ndt的初始位姿，默认没有得到位姿
  ros::NodeHandle nh_, private_nh_;

  ros::Subscriber initial_guess_sub_;
  ros::Subscriber map_points_sub_;
  ros::Subscriber sub_him_flag;
  ros::Subscriber subGps;
  ros::Subscriber subLaserCloud; // 当前帧点云接收

  ros::Publisher sensor_aligned_pose_pub_;
  ros::Publisher ndt_pose_pub_;
  ros::Publisher exe_time_pub_;
  ros::Publisher transform_probability_pub_;
  ros::Publisher iteration_num_pub_;
  ros::Publisher diagnostics_pub_;
  ros::Publisher lio_initial_pose_pub_;

  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;

  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;
  tf2_ros::TransformBroadcaster tf2_broadcaster_;

  Eigen::Matrix4f base_to_sensor_matrix_;
  Eigen::Matrix4f pre_trans, delta_trans;

  // 从gps中获取ndt的初始位姿
  // 该初始位姿指的是第一帧ndt的initial guess
  geometry_msgs::PoseStamped ndtInitPoseMsg;

  int the_num = 1;

  // gps
  std::mutex mtxGPS;
  ros::Subscriber subGPS;
  std::deque<nav_msgs::Odometry> gpsQueue;                                               // GPS原始数据队列

  // 从gps得到当前位姿
  geometry_msgs::PoseWithCovarianceStamped initPoseCovMsg;

  void gpsHandler(const nav_msgs::Odometry::ConstPtr &gpsMsg);                           // GPS回调函数


  void ndtInitPose(const nav_msgs::Odometry &thisGpsMsg); // GPS初定位


  nav_msgs::Odometry initial_guess;
  bool ndt_init = false;
  bool getinitpose = true;

  std::mutex ndt_map_mtx_;

  double converged_param_transform_probability_;
  std::map<std::string, std::string> key_value_stdmap_;

  // function
  void init_params();

  bool get_transform(const std::string &target_frame, const std::string &source_frame,
                     const geometry_msgs::TransformStamped::Ptr &transform_stamped_ptr,
                     const ros::Time &time_stamp);
  bool get_transform(const std::string &target_frame,
                     const std::string &source_frame,
                     const geometry_msgs::TransformStamped::Ptr &transform_stamped_ptr);
  void publish_tf(const std::string &frame_id, const std::string &child_frame_id,
                  const geometry_msgs::PoseStamped &pose_msg);
  void callback_pointsmap(const sensor_msgs::PointCloud2::ConstPtr &pointcloud2_msg_ptr);
  void callback_init_guess(const nav_msgs::Odometry::ConstPtr &odomMsg);
  void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);

  pcl::PointCloud<pcl::PointXYZ> removePointsByRange(pcl::PointCloud<pcl::PointXYZ> scan, double min_range, double max_range);

}; // NdtLocalizer Core
#endif