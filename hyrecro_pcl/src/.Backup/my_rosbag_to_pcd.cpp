#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <boost/filesystem.hpp>
#include <sstream>
#include <string>
#include <pcl_ros/transforms.h>

typedef sensor_msgs::PointCloud2 PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudConstPtr;

/* ---[ */
int
main(int argc, char ** argv)
{
  ros::init(argc, argv, "bag_to_pcd");
  if (argc < 4) {
    std::cerr << "Syntax is: " << argv[0] <<
      " <file_in.bag> <topic> <output_directory> [<target_frame>]" << std::endl;
    std::cerr << "Example: " << argv[0] << " data.bag /laser_tilt_cloud ./pointclouds /base_link" <<
      std::endl;
    return -1;
  }

  // TF
  tf::TransformListener tf_listener;
  tf::TransformBroadcaster tf_broadcaster;

  rosbag::Bag bag;
  rosbag::View view;
  rosbag::View::iterator view_it;
  int count = 0;

  try {
    bag.open(argv[1], rosbag::bagmode::Read);
  } catch (rosbag::BagException) {
    std::cerr << "Error opening file " << argv[1] << std::endl;
    return -1;
  }

  view.addQuery(bag, rosbag::TypeQuery("sensor_msgs/PointCloud2"));
  view_it = view.begin();

  std::string output_dir = std::string(argv[3]);
  boost::filesystem::path outpath(output_dir);
  if (!boost::filesystem::exists(outpath)) {
    if (!boost::filesystem::create_directories(outpath)) {
      std::cerr << "Error creating directory " << output_dir << std::endl;
      return -1;
    }
    std::cerr << "Creating directory " << output_dir << std::endl;
  }

  // Add the PointCloud2 handler
  std::cerr << "Saving recorded sensor_msgs::PointCloud2 messages on topic " << argv[2] << " to " <<
    output_dir << std::endl;

  PointCloud cloud_t;
  ros::Duration r(0.001);
  // Loop over the whole bag file
  while (view_it != view.end()) {

      // Get the PointCloud2 message
      PointCloudConstPtr cloud = view_it->instantiate<PointCloud>();
      if (cloud == NULL) {
        ++view_it;
        continue;
      }

      cloud_t = *cloud;

      std::cerr << "Got " << cloud_t.width * cloud_t.height << " data points in frame " <<
        cloud_t.header.frame_id << " with the following fields: " << pcl::getFieldsList(cloud_t) <<
        std::endl;

      std::stringstream ss;
      ss << output_dir << "/" << count << ".pcd";
      count++;
      std::cerr << "Data saved to " << ss.str() << std::endl;
      pcl::io::savePCDFile(
        ss.str(), cloud_t, Eigen::Vector4f::Zero(),
        Eigen::Quaternionf::Identity(), true);

      ++view_it;
    }


  return 0;
}