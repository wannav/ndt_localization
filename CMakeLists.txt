cmake_minimum_required(VERSION 3.0.2)
project(ndt_localization)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

find_package(PCL REQUIRED QUIET)

catkin_package(
  INCLUDE_DIRS include

  CATKIN_DEPENDS
  geometry_msgs 
  nav_msgs 
  pcl_conversions 
  pcl_ros 
  roscpp 
  rospy 
  sensor_msgs 
  std_msgs 
  tf
)

include_directories(include/ndt_localization ${catkin_INCLUDE_DIRS})

include_directories( "/usr/include/eigen3" )

add_executable(${PROJECT_NAME}_mapLoader src/mapLoader.cpp)
target_link_libraries(${PROJECT_NAME}_mapLoader ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(${PROJECT_NAME}_ndt src/ndt.cpp)
target_link_libraries(${PROJECT_NAME}_ndt ${catkin_LIBRARIES} ${PCL_LIBRARIES})