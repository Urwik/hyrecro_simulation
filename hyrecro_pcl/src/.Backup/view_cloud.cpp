/*
This node reads a .pcd file and plot the cloud in a new window using the 
most simple visualizer in the pcl/visualization library (CloudViewer).

USAGE: rosrun pcl_hyrecro view_cloud PATH_TO_FILE/FILE_NAME
       eg: rosrun pcl_hyrecro view_cloud /home/fran/hyrecro_ws/src/pcl_hyrecro/dataSets/set1/1.pcd
       
       If your current location is already the folder where the .pcd file is located you only need
       to specify the .pcd file
*/

// cpp
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>

// ROS 
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/visualization/cloud_viewer.h>


// Type Definitions ////////////////////////////////////////////////////////////
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;


//Global Variables

std::string getExtension(std::string input){
  std::string extension;

  std::size_t ext_start = input.find_last_of(".");
  extension = input.substr(ext_start);

  return extension;
}

int main(int argc, char **argv)
{
  //ROS Stuff
  ros::init(argc, argv, "actual_cloud_viewer");
  ros::NodeHandle n;

  // PCL Stuff
  pcl::visualization::CloudViewer viewer ("Cloud Viewer");
  pcl::PCDReader reader;

  std::string filename = argv[1];
  PointCloud::Ptr cloud (new PointCloud);

  if (argc != 2)
    ROS_ERROR("This node only takes 1 argument, the name of the .pcd file including extension");

  else if (getExtension(filename) != ".pcd")
    ROS_ERROR("The file should has .pcd extension");
  else {
    reader.read(filename, *cloud);
    ROS_INFO("Showing %s", filename.c_str());
    ROS_INFO("Cloud Points: %ld", cloud->size());
    viewer.showCloud(cloud);
    while(n.ok() && !viewer.wasStopped()){}
  }
  
  return 0;
}

