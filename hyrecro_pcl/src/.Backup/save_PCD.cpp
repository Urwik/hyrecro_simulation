// cpp
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>

// ROS 
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>


// Type Definitions ////////////////////////////////////////////////////////////
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;


//Global Variables
bool saved = 0;
PointCloud::Ptr cloud (new PointCloud);


void pcCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  // Do Once
  //if (saved == 0){  
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);
    //pcl::PCDWriter writer;

    PointCloud::Ptr temp_cloud (new PointCloud);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

    *cloud = *temp_cloud;

    //writer.write<PointT>("data7.pcd", *temp_cloud, false);

    //saved = 1;
    //std::cout << "File Saved" << '\n';
  //}

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "cloud_writer");
  ros::NodeHandle n;
  ros::Rate loop(10);
  pcl::PCDWriter writer;
  char key;
  int count=0;
  std::stringstream ss;

  while(ros::ok()){
    ros::Subscriber sub = n.subscribe("/os1/pointCloud", 1, pcCallback);
    std::cout << "Press 'c' to continue " << std::endl;
    std::cin >> key;
    if(key == 'c' && !cloud->empty()){
      ss.str("");
      ss << "data" << count <<".pcd";
      writer.write<PointT>(ss.str(), *cloud, false);
      count++;
    }
    else
      std::cout << "Cloud Its Empty" << std::endl;
    ros::spinOnce();
    loop.sleep();
  }
  return 0;
}

