/*
This node reads a topic of type sensor_msgs/PointCloud2 and plot the most recent msg 
in a new window using the most simple visualizer in the pcl/visualization library (CloudViewer).

USAGE: rosrun pcl_hyrecro actual_cloud_viewer TOPIC_NAME
       eg: rosrun pcl_hyrecro actual_cloud_viewer /os1/pointCloud
       If no topic is specified default topic is /os1/pointCloud
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
pcl::visualization::CloudViewer viewer ("Cloud Viewer");

void pcCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input, pcl_pc2);

  PointCloud::Ptr temp_cloud (new PointCloud);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

  viewer.showCloud(temp_cloud);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "actual_cloud_viewer");
  ros::NodeHandle n;
  ros::Rate loop(5);

  std::string topicName;

  if (argc == 1)
    topicName = "/os1/pointCloud";
  else if (argc == 2)
    topicName = argv[1];
  else 
    ROS_ERROR("Usage: \n showCloud /os1/pointCloud");
    
  ros::Subscriber sub = n.subscribe(topicName, 1, pcCallback);

  ros::spin();

  return 0;
}

