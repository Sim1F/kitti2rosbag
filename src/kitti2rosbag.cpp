#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <filesystem>
#include <sys/time.h>

using namespace std;

using ContainerType = std::vector<Eigen::Vector3d>;
using ContainerTypePtr = std::shared_ptr<ContainerType>;

int main(int argc, char** argv) {
  ros::init(argc, argv, "kitti2rosbag_node");

  std::string param;
  ros::NodeHandle nh("~");
  
  // Choose your desired KITTI input folder
  std::string path_to_input = "/media/simone/PortableSSD/datasets/kitti/data_odometry_velodyne/dataset/sequences/02/velodyne";

  rosbag::Bag bag;

  // Choose your desired output bag file name
  std::string bag_file_name = "/media/simone/PortableSSD/todelete.bag";
  bag.open(bag_file_name, rosbag::bagmode::Write);

  std::vector<std::filesystem::path> files_in_directory;
  std::copy(std::filesystem::directory_iterator(path_to_input),
            std::filesystem::directory_iterator(),
            std::back_inserter(files_in_directory));
  std::sort(files_in_directory.begin(), files_in_directory.end());

  const int tot_clouds = files_in_directory.size();
  cerr << "files number: " << tot_clouds << endl;

  std::vector<double> stamps;

  double time = 0.;
  const double sensor_hz = 10.0; // You should set the appropriate sensor_hz
  const double time_incr = 1.0 / sensor_hz;
  int current_cloud = 0;

  for (const std::string& filename : files_in_directory) {
    if (!ros::ok())
      break;

    // allocate 4 MB buffer (only ~130*4*4 KB are needed)
    int32_t num = 1000000;
    float* data = (float*) malloc(num * sizeof(float));

    // pointers
    float* px = data + 0;
    float* py = data + 1;
    float* pz = data + 2;
    float* pr = data + 3;

    // ContainerTypePtr points = std::make_shared<ContainerType>();
    // points->reserve(num);
    // load point cloud
    FILE* stream;
    stream = fopen(filename.c_str(), "rb");
    num    = fread(data, sizeof(float), num, stream) / 4;
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    for (int32_t i = 0; i < num; i++) {
      pcl::PointXYZI point;
      point.x = *px;
      point.y = *py;
      point.z = *pz;
      point.intensity = *pr;

      px += 4;
      py += 4;
      pz += 4;
      pr += 4;

      // if (point.norm() < min_range
      //    || point.norm() > max_range
      //     || std::isnan(point.x())
      //     || std::isnan(point.y())
      //     || std::isnan(point.z())) 
      //     continue; 

      if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
        continue;
      }

      pcl_cloud.push_back(point);
      
    }

    fclose(stream);
    free(data);

    cerr << "processed cloud " << current_cloud << "/" << tot_clouds << ", points number: " << pcl_cloud.size() << endl;
    current_cloud++;

    // Create a sensor_msgs::PointCloud2 message
    sensor_msgs::PointCloud2 pcl_msg;
    pcl::toROSMsg(pcl_cloud, pcl_msg);
    pcl_msg.header.stamp = ros::Time(time);
    pcl_msg.header.frame_id = "your_frame_id"; // Set the appropriate frame_id

    stamps.push_back(time);
    time += time_incr;

    // Write the point cloud message to the ROS bag
    bag.write("/point_cloud_topic", ros::Time(time), pcl_msg);


  }

  // Close the ROS bag
  bag.close();

  return 0;
}