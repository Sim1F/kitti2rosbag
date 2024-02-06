add_executable(kitti2rosbag kitti2rosbag.cpp)
target_link_libraries(kitti2rosbag
${catkin_LIBRARIES} 
${rosbag_LIBRARIES} 
${PCL_LIBRARIES})