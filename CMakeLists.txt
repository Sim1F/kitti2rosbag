cmake_minimum_required(VERSION 3.0.2)
project(kitti2rosbag)

set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE RelWithDebInfo)
#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS -Werror)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rosbag
  tf2_ros
)

find_package(PCL REQUIRED)

catkin_package(INCLUDE_DIRS src LIBRARIES kitti2rosbag)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${rosbag_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/src
  "/usr/include/eigen3"
)

add_subdirectory(src)