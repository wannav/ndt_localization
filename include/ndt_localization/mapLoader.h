#ifndef _MAP_LOADER_H_
#define _MAP_LOADER_H_

#include <vector>
#include <numeric>
#include <chrono>
#include <iomanip>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

class MapLoader
{
public:
  MapLoader(ros::NodeHandle &nh);
  std::vector<std::string> file_list_;

private:
  float tf_x_, tf_y_, tf_z_, tf_roll_, tf_pitch_, tf_yaw_;

  std::string map_topic;
  std::string mapFilePath;

  // 加载地图
  ros::Publisher pubMap;
  std::vector<double> v_mapLoadingTime;

  void init_tf_params(ros::NodeHandle &nh);

  sensor_msgs::PointCloud2 CreatePcd();
  sensor_msgs::PointCloud2 loadMap();

  pcl::PointCloud<pcl::PointXYZ> removePointsByRange(pcl::PointCloud<pcl::PointXYZ> scan, double min_range, double max_range);

}; // MapLoader

#endif