#include "ndt.h"

ParamServer::ParamServer()
{
  std::cout << "-------ParamServer---------" << std::endl;

  // topics
  nh.param<std::string>("ndt_localization/pointCloudTopic", pointCloudTopic, "/lslidar_point_cloud");
  nh.param<std::string>("ndt_localization/gpsTopic", gpsTopic, "/GPS_odometry");
  nh.param<std::string>("ndt_localization/mapTopic", mapTopic, "/points_map");

  // frames
  nh.param<std::string>("ndt_localization/baseFrame", baseFrame, "base_link");
  nh.param<std::string>("ndt_localization/gpsFrame", gpsFrame, "GPS");
  nh.param<std::string>("ndt_localization/mapFrame", mapFrame, "map");

  // frequence control
  nh.param<float>("ndt_localization/mappingProcessInterval", mappingProcessInterval, 0.15);

  // ndt
  nh.param<double>("ndt_localization/trans_epsilon", trans_epsilon, 0.0);
  nh.param<double>("ndt_localization/step_size", step_size, 0.0);
  nh.param<double>("ndt_localization/resolution", resolution, 0.0);
  nh.param<int>("ndt_localization/max_iterations", max_iterations, 0.0);
  nh.param<double>("ndt_localization/converged_param_transform_probability", converged_param_transform_probability, 0.0);
  nh.param<double>("ndt_localization/vg_scan_size", vg_scan_size, 0.0);
  nh.param<double>("ndt_localization/radius", radius, 0.0);
  ROS_INFO("trans_epsilon: %lf, step_size: %lf, resolution: %lf, max_iterations: %d", trans_epsilon, step_size, resolution, max_iterations);

  // gps
  nh.param<double>("ndt_localization/gpsCovThreshold", gpsCovThreshold, 0.0);

  // delay time
  nh.param<int>("ndt_localization/start_delay_time", ndtDelayTime, 10);
}

NdtLocalizer::NdtLocalizer(ros::NodeHandle &nh, ros::NodeHandle &private_nh) : nh_(nh), private_nh_(private_nh), tf2_listener_(tf2_buffer_)
{
  std::cout << "-------NdtLocalizer---------" << std::endl;

  key_value_stdmap_["state"] = "Initializing";
  init_params();

  // Publishers
  sensor_aligned_pose_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("points_aligned", 10);
  ndt_pose_pub_ = nh_.advertise<nav_msgs::Odometry>("ndt_pose", 10);
  exe_time_pub_ = nh_.advertise<std_msgs::Float32>("exe_time_ms", 10);
  transform_probability_pub_ = nh_.advertise<std_msgs::Float32>("transform_probability", 10);
  iteration_num_pub_ = nh_.advertise<std_msgs::Float32>("iteration_num", 10);
  diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 10);
  lio_initial_pose_pub_ = nh_.advertise<nav_msgs::Odometry>("lio_initial_pose", 10, true);

  // Subscribers
  map_points_sub_ = nh_.subscribe(mapTopic, 1, &NdtLocalizer::callback_pointsmap, this);
  subGps = nh_.subscribe(gpsTopic, 2000, &NdtLocalizer::gpsHandler, this);
  subLaserCloud = nh_.subscribe(pointCloudTopic, 2000, &NdtLocalizer::cloudHandler, this);

  try
  {
    tfListener.waitForTransform(baseFrame, gpsFrame, ros::Time(0), ros::Duration(3.0));
    tfListener.lookupTransform(baseFrame, gpsFrame, ros::Time(0), Baselink2GPS);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }
}

NdtLocalizer::~NdtLocalizer() {}

void NdtLocalizer::gpsHandler(const nav_msgs::Odometry::ConstPtr &gpsMsg)
{
  std::cout << "-------gpsHandler---------" << std::endl;

  // std::lock_guard<std::mutex> lock(mtxGPS);
  nav_msgs::Odometry this_gps, this_gps_to_lidar;
  this_gps = *gpsMsg;

  float noise_x = this_gps.pose.covariance[0];
  float noise_y = this_gps.pose.covariance[7];
  // float noise_z = this_gps.pose.covariance[14];
  if (noise_x > 0.01 || noise_y > 0.01)
  {
    std::cout << "GPS too noisy, skip" << std::endl;
    return;
  }
  // gpsQueue.push_back(this_gps);

  tf::Transform t_gps;
  tf::poseMsgToTF(this_gps.pose.pose, t_gps);
  // 转换至Lidar坐标系下的GPS数据
  t_gps = t_gps * Baselink2GPS.inverse();
  this_gps_to_lidar.pose.pose.position.x = t_gps.getOrigin().x();
  this_gps_to_lidar.pose.pose.position.y = t_gps.getOrigin().y();
  this_gps_to_lidar.pose.pose.position.z = t_gps.getOrigin().z();
  this_gps_to_lidar.pose.pose.orientation.x = t_gps.getRotation().x();
  this_gps_to_lidar.pose.pose.orientation.y = t_gps.getRotation().y();
  this_gps_to_lidar.pose.pose.orientation.z = t_gps.getRotation().z();
  this_gps_to_lidar.pose.pose.orientation.w = t_gps.getRotation().w();
  std::cout << "转换后的gps child frame id: " << this_gps_to_lidar.child_frame_id << std::endl;

  // geometry_msgs::PoseWithCovarianceStamped::Ptr gpsPoseMsgPtr(new geometry_msgs::PoseWithCovarianceStamped);
  // gpsPoseMsgPtr->header = this_gps_to_lidar.header;
  // gpsPoseMsgPtr->header.frame_id = baseFrame; //可以跟据上面cout的frameid是什么，看看要不要加这行
  // gpsPoseMsgPtr->pose = this_gps_to_lidar.pose;

  // if (getinitpose)
  // {
  ndtInitPoseMsg.header.frame_id = baseFrame;
  ndtInitPoseMsg.header.stamp = this_gps_to_lidar.header.stamp;
  ndtInitPoseMsg.pose = this_gps_to_lidar.pose.pose;
  getinitpose = false;
  // }

  // 打印gps坐标 baselink
  gpsOdomFile << std::setprecision(15);
  gpsOdomFile << this_gps_to_lidar.header.stamp << " "
              << this_gps_to_lidar.pose.pose.position.x << " "
              << this_gps_to_lidar.pose.pose.position.y << " "
              << this_gps_to_lidar.pose.pose.position.z << " "
              << this_gps_to_lidar.pose.pose.orientation.x << " "
              << this_gps_to_lidar.pose.pose.orientation.y << " "
              << this_gps_to_lidar.pose.pose.orientation.z << " "
              << this_gps_to_lidar.pose.pose.orientation.w << std::endl;
}

// void NdtLocalizer::ndtInitPose(const nav_msgs::Odometry &thisGpsMsg)
// {
//   ndtInitPoseMsg.header = thisGpsMsg.header;
//   ndtInitPoseMsg.pose = thisGpsMsg.pose.pose;
// }

void NdtLocalizer::callback_pointsmap(const sensor_msgs::PointCloud2::ConstPtr &mapMsgPtr)
{
  std::cout << "-----in callback_pointsmap------------" << std::endl;
  // InitDelay(ndtDelayTime);
  // const auto trans_epsilon = ndt_.getTransformationEpsilon();
  // const auto step_size = ndt_.getStepSize();
  // const auto resolution = ndt_.getResolution();
  // const auto max_iterations = ndt_.getMaximumIterations();

  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_new;

  ndt_new.setTransformationEpsilon(trans_epsilon);
  ndt_new.setStepSize(step_size);
  ndt_new.setResolution(resolution);
  ndt_new.setMaximumIterations(max_iterations);

  pcl::PointCloud<pcl::PointXYZ>::Ptr mapPtr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*mapMsgPtr, *mapPtr);
  ndt_new.setInputTarget(mapPtr);
  // create Thread
  // detach
  pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>);
  ndt_new.align(*outputCloud, Eigen::Matrix4f::Identity());

  // swap
  ndt_map_mtx_.lock();
  ndt_ = ndt_new;
  ndt_map_mtx_.unlock();
}

void NdtLocalizer::cloudHandler(const sensor_msgs::PointCloud2::ConstPtr &laserCloudMsg)
{
  std::cout << "-----------cloudHandler-------------" << std::endl;

  std::lock_guard<std::mutex> lock(ndt_map_mtx_);

  /******************************************************
  1. save frame and time
  ******************************************************/
  const std::string sensorFrame = baseFrame; // 在我们的车上baselink就是lidarlink
  // const std::string sensorFrame = lidarFrame;
  const auto sensorRosTime = laserCloudMsg->header.stamp;

  pcl::PointCloud<pcl::PointXYZ>::Ptr thisScanPtr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*laserCloudMsg, *thisScanPtr);

  // no map
  if (ndt_.getInputTarget() == nullptr)
  {
    std::cout << "no map!" << std::endl;

    ROS_WARN_STREAM_THROTTLE(1, "No MAP!");
    return;
  }

  //******************************************************
  // 2. 将scan从base_link转换到GPS
  //******************************************************
  // for (const pcl::PointXYZ &thisScanPoint : *thisScanPtr)
  // {
  //   tf::Point inPoint(thisScanPoint.x, thisScanPoint.y, thisScanPoint.z);
  //   tf::Point outPoint = inPoint * Baselink2GPS;
  // }

  /******************************************************
  3. Down sample the laser points
  In:   cloudInfoPtr
  out:  sensorlinkcloudPtr
  ******************************************************/
  // if (measurement_range != MAX_MEASUREMENT_RANGE)
  // {
  //   *thisScanPtr = removePointsByRange(*thisScanPtr, 0, measurement_range);
  // }

  pcl::PointCloud<pcl::PointXYZ>::Ptr filteredScanPtr(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr sensorlinkcloudPtr(new pcl::PointCloud<pcl::PointXYZ>());
  // if voxel_leaf_size < 0.1 voxel_grid_filter cannot down sample (It is specification in PCL)
  if (vg_scan_size >= 0.1)
  {
    // Downsampling the velodyne scan using VoxelGrid filter
    pcl::VoxelGrid<pcl::PointXYZ> voxelGridFilter;
    voxelGridFilter.setLeafSize(vg_scan_size, vg_scan_size, vg_scan_size);
    voxelGridFilter.setInputCloud(thisScanPtr);
    voxelGridFilter.filter(*filteredScanPtr);
    sensorlinkcloudPtr = filteredScanPtr;
  }
  else
  {
    sensorlinkcloudPtr = thisScanPtr;
  }

  /******************************************************
  4. tf the laser points form sensor frame to base_link and input the lidar points to NDT
  In: sensorlinkcloudPtr
  ******************************************************/
  const auto exe_start_time = std::chrono::system_clock::now();
  // // get TF base to sensor
  // geometry_msgs::TransformStamped::Ptr basetosensorTFPtr(new geometry_msgs::TransformStamped);
  // get_transform(baseFrame, sensorFrame, basetosensorTFPtr);
  // // get_transform(baseFrame, gpsFrame, basetosensorTFPtr);
  // // get_transform(gpsFrame, baseFrame, basetosensorTFPtr);

  // const Eigen::Affine3d base_to_sensor_affine = tf2::transformToEigen(*basetosensorTFPtr);
  // const Eigen::Matrix4f base_to_sensor_matrix = base_to_sensor_affine.matrix().cast<float>();

  // // boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> sensor_points_baselinkTF_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr sensor_points_baselinkTF_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::transformPointCloud(*sensorlinkcloudPtr, *sensor_points_baselinkTF_ptr, base_to_sensor_matrix);
  // set input point cloud
  // ndt_.setInputSource(sensor_points_baselinkTF_ptr);

  // 不进行坐标变换，即在baselink下配准
  ndt_.setInputSource(sensorlinkcloudPtr);

  /******************************************************
  5. initial guess
  ******************************************************/
  Eigen::Matrix4f initGuessMatrix;
  if (!initPose)
  {
    Eigen::Affine3d initGuessAffine;
    tf2::fromMsg(ndtInitPoseMsg.pose, initGuessAffine);
    initGuessMatrix = initGuessAffine.matrix().cast<float>();
    // for the first time, we don't know the pre_trans, so just use the init_trans,
    // which means, the delta trans for the second time is 0
    pre_trans = initGuessMatrix;
    initPose = true;
  }
  else
  {
    // use predicted pose as init guess (currently we only impl linear model)
    initGuessMatrix = pre_trans * delta_trans;
  }

  /******************************************************
  5. Do NDT
  In: Eigen::Matrix4f initGuessMatrix
  Out: geometry_msgs::Pose result_pose_msg
  ******************************************************/
  pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>);
  const auto align_start_time = std::chrono::system_clock::now();
  key_value_stdmap_["state"] = "Aligning";
  ndt_.align(*outputCloud, initGuessMatrix);
  key_value_stdmap_["state"] = "Sleeping";
  const auto align_end_time = std::chrono::system_clock::now();
  const double align_time = std::chrono::duration_cast<std::chrono::microseconds>(align_end_time - align_start_time).count() / 1000.0;

  const Eigen::Matrix4f result_pose_matrix = ndt_.getFinalTransformation();
  Eigen::Affine3d result_pose_affine;
  result_pose_affine.matrix() = result_pose_matrix.cast<double>();
  const geometry_msgs::Pose result_pose_msg = tf2::toMsg(result_pose_affine);

  const auto exe_end_time = std::chrono::system_clock::now();
  const double exe_time = std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count() / 1000.0;

  if (usrDebug == true)
  {
    std::cout << "result_pose_msg" << std::endl;
    std::cout << result_pose_msg << std::endl;
  }
  /******************************************************
  6. Handle the result
  ******************************************************/
  const float transform_probability = ndt_.getTransformationProbability();
  const int iteration_num = ndt_.getFinalNumIteration();

  bool is_converged = true;
  static size_t skipping_publish_num = 0;
  if (iteration_num >= ndt_.getMaximumIterations() + 2 || transform_probability < converged_param_transform_probability_)
  {
    is_converged = false;
    // initPose = false;
    ++skipping_publish_num;
    std::cout << "--------------------------------------Not Converged" << std::endl;
  }
  else
  {
    skipping_publish_num = 0;
  }
  // calculate the delta tf from pre_trans to current_trans
  delta_trans = pre_trans.inverse() * result_pose_matrix;

  Eigen::Vector3f delta_translation = delta_trans.block<3, 1>(0, 3);
  std::cout << "delta x: " << delta_translation(0) << " y: " << delta_translation(1) << " z: " << delta_translation(2) << std::endl;

  Eigen::Matrix3f delta_rotation_matrix = delta_trans.block<3, 3>(0, 0);
  Eigen::Vector3f delta_euler = delta_rotation_matrix.eulerAngles(2, 1, 0);
  std::cout << "delta yaw: " << delta_euler(0) << " pitch: " << delta_euler(1) << " roll: " << delta_euler(2) << std::endl;

  pre_trans = result_pose_matrix;

  /******************************************************
  7. publish the topic
  pub:
  ******************************************************/
  geometry_msgs::PoseStamped result_pose_stamped_msg;
  result_pose_stamped_msg.header.stamp = sensorRosTime;
  result_pose_stamped_msg.header.frame_id = mapFrame;
  result_pose_stamped_msg.pose = result_pose_msg;

  ndtPoseFile << std::setprecision(15);
  ndtPoseFile << result_pose_stamped_msg.header.stamp.toSec() << " "
              << result_pose_stamped_msg.pose.position.x << " " << result_pose_stamped_msg.pose.position.y << " "
              << result_pose_stamped_msg.pose.position.z << " "
              << result_pose_stamped_msg.pose.orientation.x << " " << result_pose_stamped_msg.pose.orientation.y << " "
              << result_pose_stamped_msg.pose.orientation.z << " " << result_pose_stamped_msg.pose.orientation.w << std::endl;

  if (is_converged)
  {
    ndt_pose_pub_.publish(result_pose_stamped_msg);
  }

  // publish tf(map frame to base frame)
  publish_tf(mapFrame, baseFrame, result_pose_stamped_msg);

  // publish aligned point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr sensor_points_mapTF_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::transformPointCloud(*sensor_points_baselinkTF_ptr, *sensor_points_mapTF_ptr, result_pose_matrix);

  // 不进行坐标变换，即在baselink下配准
  pcl::transformPointCloud(*sensorlinkcloudPtr, *sensor_points_mapTF_ptr, result_pose_matrix);
  sensor_msgs::PointCloud2 sensor_points_mapTF_msg;
  pcl::toROSMsg(*sensor_points_mapTF_ptr, sensor_points_mapTF_msg);
  sensor_points_mapTF_msg.header.stamp = sensorRosTime;
  sensor_points_mapTF_msg.header.frame_id = mapFrame;
  sensor_aligned_pose_pub_.publish(sensor_points_mapTF_msg);

  std_msgs::Float32 exe_time_msg;
  exe_time_msg.data = exe_time;
  exe_time_pub_.publish(exe_time_msg);

  std_msgs::Float32 transform_probability_msg;
  transform_probability_msg.data = transform_probability;
  transform_probability_pub_.publish(transform_probability_msg);

  std_msgs::Float32 iteration_num_msg;
  iteration_num_msg.data = iteration_num;
  iteration_num_pub_.publish(iteration_num_msg);

  key_value_stdmap_["seq"] = std::to_string(laserCloudMsg->header.seq);
  key_value_stdmap_["transform_probability"] = std::to_string(transform_probability);
  key_value_stdmap_["iteration_num"] = std::to_string(iteration_num);
  key_value_stdmap_["skipping_publish_num"] = std::to_string(skipping_publish_num);

  std::cout << "------------------------------------------------" << std::endl;
  std::cout << "align_time: " << align_time << "ms" << std::endl;
  std::cout << "exe_time: " << exe_time << "ms" << std::endl;
  std::cout << "trans_prob: " << transform_probability << std::endl;
  std::cout << "iter_num: " << iteration_num << std::endl;
  std::cout << "skipping_publish_num: " << skipping_publish_num << std::endl;
}

void NdtLocalizer::init_params()
{
  private_nh_.param<std::string>("ndt_localization/outputPath", outputPath, " ");

  ndtPoseFile.open("/home/w/ndt_localization/src/ndt_localization/output/ndtPose.txt");
  gpsOdomFile.open("/home/w/ndt_localization/src/ndt_localization/output//gpsPose.txt");

  initPose = false;

  ndt_.setTransformationEpsilon(trans_epsilon);
  ndt_.setStepSize(step_size);
  ndt_.setResolution(resolution);
  ndt_.setMaximumIterations(max_iterations);

  ROS_INFO(
      "trans_epsilon: %lf, step_size: %lf, resolution: %lf, max_iterations: %d", trans_epsilon,
      step_size, resolution, max_iterations);

  private_nh_.getParam("converged_param_transform_probability", converged_param_transform_probability_);
}

bool NdtLocalizer::get_transform(
    const std::string &target_frame, const std::string &source_frame,
    const geometry_msgs::TransformStamped::Ptr &transform_stamped_ptr, const ros::Time &time_stamp)
{
  if (target_frame == source_frame)
  {
    transform_stamped_ptr->header.stamp = time_stamp;
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return true;
  }

  try
  {
    *transform_stamped_ptr = tf2_buffer_.lookupTransform(target_frame, source_frame, time_stamp);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    ROS_ERROR("Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

    transform_stamped_ptr->header.stamp = time_stamp;
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return false;
  }
  return true;
}

bool NdtLocalizer::get_transform(const std::string &target_frame,
                                 const std::string &source_frame,
                                 const geometry_msgs::TransformStamped::Ptr &transform_stamped_ptr)
{
  if (target_frame == source_frame)
  {
    transform_stamped_ptr->header.stamp = ros::Time::now();
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return true;
  }

  try
  {
    *transform_stamped_ptr = tf2_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    ROS_ERROR("Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

    transform_stamped_ptr->header.stamp = ros::Time::now();
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return false;
  }
  return true;
}

void NdtLocalizer::publish_tf(
    const std::string &frame_id,
    const std::string &child_frame_id,
    const geometry_msgs::PoseStamped &pose_msg)
{
  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = frame_id;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.header.stamp = pose_msg.header.stamp;

  transform_stamped.transform.translation.x = pose_msg.pose.position.x;
  transform_stamped.transform.translation.y = pose_msg.pose.position.y;
  transform_stamped.transform.translation.z = pose_msg.pose.position.z;

  tf2::Quaternion tf_quaternion;
  tf2::fromMsg(pose_msg.pose.orientation, tf_quaternion);
  transform_stamped.transform.rotation.x = tf_quaternion.x();
  transform_stamped.transform.rotation.y = tf_quaternion.y();
  transform_stamped.transform.rotation.z = tf_quaternion.z();
  transform_stamped.transform.rotation.w = tf_quaternion.w();

  tf2_broadcaster_.sendTransform(transform_stamped);
}

pcl::PointCloud<pcl::PointXYZ> NdtLocalizer::removePointsByRange(pcl::PointCloud<pcl::PointXYZ> scan, double min_range, double max_range)
{
  pcl::PointCloud<pcl::PointXYZ> narrowed_scan;
  narrowed_scan.header = scan.header;

#if 1 //  This error handling should be detemind.
  if (min_range >= max_range)
  {
    ROS_ERROR_ONCE("min_range>=max_range @(%lf, %lf)", min_range, max_range);
    return scan;
  }
#endif

  double square_min_range = min_range * min_range;
  double square_max_range = max_range * max_range;

  for (pcl::PointCloud<pcl::PointXYZ>::const_iterator iter = scan.begin(); iter != scan.end(); ++iter)
  {
    const pcl::PointXYZ &p = *iter;
    //    p.x = iter->x;
    //    p.y = iter->y;
    //    p.z = iter->z;
    //    p.intensity = iter->intensity;
    double square_distance = p.x * p.x + p.y * p.y;

    if (square_min_range <= square_distance && square_distance <= square_max_range)
    {
      narrowed_scan.points.push_back(p);
    }
  }

  return narrowed_scan;
}

//延时
void NdtLocalizer::InitDelay(int delaySec)
{
  ros::Rate loop_rate(100); // 100Hz
  unsigned int time_count = 0;
  while (ros::ok())
  {
    if (sys_status == sys_none)
    {
      time_count++;
      // cout << "time_count = " << time_count << endl;
      if (time_count >= 1 * delaySec)
      {
        time_count = 0;
        sys_status = sys_init_pose;
        std::cout << "sys_status = sys_init_pose !!!" << std::endl;
      }
    }
    loop_rate.sleep();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ndt");

  ROS_INFO("\033[1;32m---->\033[0m NDT Started.");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  NdtLocalizer ndt_localizer(nh, private_nh);

  // std::thread InitDelay(&NdtLocalizer::InitDelay, &ndt_localizer);

  ros::spin();

  return 0;
}