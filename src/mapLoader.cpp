#include "mapLoader.h"

MapLoader::MapLoader(ros::NodeHandle &nh)
{
  std::cout << "-------MapLoader---------" << std::endl;

  // get parameter
  nh.param<std::string>("map_topic", map_topic, "/points_map");
  nh.param<std::string>("mapFilePath", mapFilePath, " ");

  // publisher
  pubMap = nh.advertise<sensor_msgs::PointCloud2>("/points_map", 10, true);
  // ros::Duration(0.5).sleep();
  // init tf parameter
  init_tf_params(nh);

  file_list_.push_back(mapFilePath);

  auto pc_msg = loadMap();

  // load map
  if (pc_msg.width != 0)
  {
    std::cout << "-------MapLoader if---------" << std::endl;
    pc_msg.header.frame_id = "GPS";
    pubMap.publish(pc_msg);
    std::cout << "finish publish!!" << std::endl;
  }
}

void MapLoader::init_tf_params(ros::NodeHandle &nh)
{
  nh.param<float>("x", tf_x_, 0.0);
  nh.param<float>("y", tf_y_, 0.0);
  nh.param<float>("z", tf_z_, 0.0);
  nh.param<float>("roll", tf_roll_, 0.0);
  nh.param<float>("pitch", tf_pitch_, 0.0);
  nh.param<float>("yaw", tf_yaw_, 0.0);
  ROS_INFO_STREAM("x" << tf_x_ << "y: " << tf_y_ << "z: " << tf_z_ << "roll: "
                      << tf_roll_ << " pitch: " << tf_pitch_ << "yaw: " << tf_yaw_);
}

sensor_msgs::PointCloud2 MapLoader::loadMap()
{
  sensor_msgs::PointCloud2 mapMsg;
  if (pcl::io::loadPCDFile(mapFilePath.c_str(), mapMsg) == -1)
  {
    std::cerr << "load failed " << mapFilePath << std::endl;
  }
  std::cerr << "load " << mapFilePath << std::endl;

  mapMsg.header.frame_id = "GPS";
  std::cout << "map size: " << mapMsg.width << std::endl;

  // 滤波
  pcl::PointCloud<pcl::PointXYZ> scan;
  pcl::fromROSMsg(mapMsg, scan);
  // scan = removePointsByRange(scan, 0, 300);

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZ>(scan));
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>());

  sensor_msgs::PointCloud2 filtered_msg;

  // if voxel_leaf_size < 0.1 voxel_grid_filter cannot down sample (It is specification in PCL)
  // Downsampling the velodyne scan using VoxelGrid filter
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(1.0, 1.0, 1.0);
  voxel_grid_filter.setInputCloud(scan_ptr);
  voxel_grid_filter.filter(*filtered_scan_ptr);
  pcl::toROSMsg(*filtered_scan_ptr, filtered_msg);
  filtered_msg.header = mapMsg.header;
  std::cout << "filtered_msg size: " << filtered_msg.width << std::endl;

  return filtered_msg;
}

pcl::PointCloud<pcl::PointXYZ> MapLoader::removePointsByRange(pcl::PointCloud<pcl::PointXYZ> scan, double min_range, double max_range)
{
  pcl::PointCloud<pcl::PointXYZ> narrowed_scan;
  narrowed_scan.header = scan.header;

  if (min_range >= max_range)
  {
    ROS_ERROR_ONCE("min_range>=max_range @(%lf, %lf)", min_range, max_range);
    return scan;
  }

  double square_min_range = min_range * min_range;
  double square_max_range = max_range * max_range;

  for (pcl::PointCloud<pcl::PointXYZ>::const_iterator iter = scan.begin(); iter != scan.end(); ++iter)
  {
    const pcl::PointXYZ &p = *iter;
    double square_distance = p.x * p.x + p.y * p.y;

    if (square_min_range <= square_distance && square_distance <= square_max_range)
    {
      narrowed_scan.points.push_back(p);
    }
  }

  return narrowed_scan;
}

sensor_msgs::PointCloud2 MapLoader::CreatePcd()
{
  sensor_msgs::PointCloud2 pcd, part;
  for (const std::string &path : file_list_)
  {
    // Following outputs are used for progress bar of Runtime Manager.
    if (pcd.width == 0)
    {
      if (pcl::io::loadPCDFile(path.c_str(), pcd) == -1)
      {
        std::cerr << "load failed " << path << std::endl;
      }
    }
    else
    {
      if (pcl::io::loadPCDFile(path.c_str(), part) == -1)
      {
        std::cerr << "load failed " << path << std::endl;
      }
      pcd.width += part.width;
      pcd.row_step += part.row_step;
      pcd.data.insert(pcd.data.end(), part.data.begin(), part.data.end());
    }
    std::cerr << "load " << path << std::endl;
    if (!ros::ok())
      break;
  }
  std::cout << "pcd size: " << pcd.width << std::endl;
  return pcd;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_loader");

  ROS_INFO("\033[1;32m---->\033[0m Map Loader Started.");

  ros::NodeHandle nh("~");

  MapLoader map_loader(nh);

  ros::spin();

  return 0;
}