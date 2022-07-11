/*
  REGISTRA POR PARES 2 NUBES DE PUNTOS ESPECIFICADAS DENTRO DEL MAIN()
*/
// cpp
#include <iostream>
#include <fstream>
#include <string>
#include <boost/filesystem.hpp>
#include <dirent.h>
#include <filesystem>

// ROS 
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>


//****************************************************************************//
// TYPE DEFINITIONS ////////////////////////////////////////////////////////////

typedef pcl::PointXYZRGB PointRGB;
typedef pcl::PointCloud<PointRGB> PointCloudRGB;

// ************************************************************************** //


// ************************************************************************** //
// AVAILABLE FUNCTIONS /////////////////////////////////////////////////////////
PointCloudRGB::Ptr output_cloud (new PointCloudRGB);
pcl::PCDWriter writer;
int count = 0;
std::stringstream ss;


void pcCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  pcl::fromROSMsg(*input, *output_cloud);

  
  ss.str("");
  ss << count <<".pcd";
  writer.write<PointRGB>(ss.str(), *output_cloud, false);

  // std::cout << "New Cloud Saved" << std::endl;
  count++;

}
  
int main(int argc, char **argv)
{
  ros::init(argc, argv, "rosbar_pointcloud_sorter");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/camera/depth/color/points", 2000, pcCallback);

  std::cout << "Service ready" << std::endl;
  ros::spin();
  return 0;
}




