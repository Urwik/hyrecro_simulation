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
typedef pcl::PointCloud<pcl::PointXYZ> pointCloud;

// Clouds
pointCloud::Ptr inputCloud (new pointCloud);

// AVAILABLE FUNCTIONS /////////////////////////////////////////////////////////
// CALLBACK //
void pcCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input, pcl_pc2);
  pointCloud::Ptr temp_cloud (new pointCloud);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

  *inputCloud = *temp_cloud;
}

// Filter the pointCloud by height (min,max)
pointCloud::Ptr filterHeight(const pointCloud::Ptr& original, float min, float max){
  pointCloud::Ptr filtered_cloud (new pointCloud);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(original);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(min, max);
  pass.filter(*filtered_cloud);

  return filtered_cloud;
}

// Downsample pointCloud by a Voxel defined with x, y, z
pointCloud::Ptr downSample(const pointCloud::Ptr& original, float x, float y, float z){
  pointCloud::Ptr ds_cloud (new pointCloud);
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(original);
  sor.setLeafSize(x, y, z);
  sor.filter(*ds_cloud);

  return ds_cloud;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "PC2_wirter");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/os1/pointCloud", 1, pcCallback);
  ros::Rate rate(1);

  pcl::PCDWriter writer;

  int count = 0;

  while(ros::ok()){
    ros::spinOnce();

    pointCloud::Ptr filtered_cloud = filterHeight(inputCloud, -0.50, 0.50);
    pointCloud::Ptr ds_cloud = downSample(filtered_cloud, 0.02f, 0.02f, 0.02f);

    std::stringstream ss;
    ss.str("");
    ss << "cloudData_" << count << ".pcd"; 

    writer.write<pcl::PointXYZ>(ss.str(), *ds_cloud, false);
    std::cout << "File Saved" << '\n';

    count++;
    rate.sleep();
  }
  return 0;
}

