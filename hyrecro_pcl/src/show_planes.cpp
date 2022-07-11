// cpp
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry> 

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
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>

  // PCL FILTERS
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>

  // SEGMENTATION
#include <pcl/segmentation/sac_segmentation.h>

  // RANSAC
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

  // PCL VISUALIZATION
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>


//****************************************************************************//
// Type Definitions ////////////////////////////////////////////////////////////
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointT> CloudColor;


// ************************************************************************** //
// Global Variables ////////////////////////////////////////////////////////////

// Cloud
PointCloud::Ptr cloud (new PointCloud);

// Visualizers
pcl::visualization::PCLVisualizer::Ptr pclVisualizer (new pcl::visualization::PCLVisualizer ("PCL Visualizer"));

// ************************************************************************** //
// AVAILABLE FUNCTIONS /////////////////////////////////////////////////////////

// Extract indices
PointCloud::Ptr extractIndices(PointCloud::Ptr original, int maxIterations, float threshold);


// Show a cloud with specific color
pcl::visualization::PCLVisualizer::Ptr customColourVis (PointCloud::ConstPtr cloud, std::string cloudName, pcl::RGB color)
{
  pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(cloud, (int) color.r, (int) color.g, (int) color.b);
  pclVisualizer->addPointCloud<PointT> (cloud, single_color, cloudName);
  pclVisualizer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, cloudName);
  pclVisualizer->spinOnce();
  return (pclVisualizer);
}

// Callback
void callback(const sensor_msgs::PointCloud2::ConstPtr& new_cloud){
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*new_cloud, pcl_pc2);

  PointCloud::Ptr temp_cloud (new PointCloud);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

  *cloud = *temp_cloud;
}

////////////////////////////////////////////////////////////////////////////////


int main(int argc, char **argv)
{
  ros::init(argc, argv, "show_planes");
  ros::NodeHandle n;
  ros::Rate rate(10);

  // Get the data  
  PointCloud::Ptr cloud (new PointCloud);
  pcl::PCDReader reader;

  std::string online = argv[1];

  // pclVisualizer->setBackgroundColor(255,255,255);
  pclVisualizer->setBackgroundColor(100,100,100);


  PointCloud::Ptr icpSource (new PointCloud);

  if (online == "true"){
    std::string topic_name = argv[2];
    ros::Subscriber cloud_sub = n.subscribe(topic_name, 10, callback);

    while (n.ok() && !pclVisualizer->wasStopped()){
      icpSource = extractIndices(cloud, 1000, 0.04);
      pclVisualizer->spinOnce (100);
      rate.sleep();
    }
  }

  else{
    std::string cloud_file = argv[3];
    reader.read(cloud_file, *cloud);
    icpSource = extractIndices(cloud, 1000, 0.04);
    while (!pclVisualizer->wasStopped()){
      pclVisualizer->spinOnce (100);
    }
    
  }

  return 0;
}



// ************************************************************************** //
// FUNCTIONS DEFINITIONS ///////////////////////////////////////////////////////

// Extract indices
PointCloud::Ptr extractIndices(PointCloud::Ptr original, int maxIterations, float threshold){
  PointCloud::Ptr centroidCloud (new PointCloud);
  PointCloud::Ptr planeCoefsCloud (new PointCloud);
  PointCloud::Ptr pCloud (new PointCloud);
  PointCloud::Ptr fCloud (new PointCloud);
  pcl::RGB rgb;


  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<PointT> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (maxIterations);
  seg.setDistanceThreshold (threshold);


  planeCoefsCloud->sensor_orientation_ = original->sensor_orientation_;
  planeCoefsCloud->sensor_origin_ = original->sensor_origin_;
  //planeCoefsCloud->clear();


  // Create the filtering object
  pcl::ExtractIndices<PointT> extract;

  int i = 0, nr_points = (int) original->size ();
  // While 30% of the original cloud is still there
  //while (original->size () > 0.1 * nr_points)
  for(i=0; i <= 10; i++)
  {

    std::stringstream ss;
    ss.str("");
    ss << "Cloud_" << i;

    std::stringstream ssP;
    ssP.str("");
    ssP << "Cloud_Plane_" << i;

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (original);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    float xn = coefficients->values[0];
    float yn = coefficients->values[1];
    float zn = coefficients->values[2];
    float wn = coefficients->values[3];

    std::cout << "Plane_" << i << ": " << xn <<"; "<< yn <<"; "<< zn << "; "<< wn <<'\n';
    
    // Extract the inliers
    extract.setInputCloud (original);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter(*pCloud);


   //**************************************
    // Generate unique colour
    rgb = pcl::GlasbeyLUT::at(i);

    //plotCloud(pCloud, ss.str());
    customColourVis(pCloud, ss.str(), rgb);
    
    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*fCloud);
    original.swap(fCloud);

    //i++;
  }
    CloudColor white (fCloud, 0, 0, 0);
    pclVisualizer->addPointCloud<PointT>(fCloud, white, "Remaining Cloud");
    pclVisualizer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "Remaining Cloud");

    std::cout << "Points Out of Planes: " << fCloud->size() << '\n';

    return fCloud; 
}




