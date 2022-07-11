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
  pclVisualizer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloudName);
  pclVisualizer->spinOnce();
  return (pclVisualizer);
}

PointCloud::Ptr getCentroidCloud(PointCloud::Ptr input, int maxIterations, float threshold){
  PointCloud::Ptr centroidCloud (new PointCloud);
  PointCloud::Ptr remainingCloud (new PointCloud);
  PointCloud::Ptr pCloud (new PointCloud);
  PointCloud::Ptr fCloud (new PointCloud);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

  // Create the segmentation object
  pcl::SACSegmentation<PointT> seg;
  
  // Optional
  seg.setOptimizeCoefficients (true);

  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(maxIterations);
  seg.setDistanceThreshold(threshold);

  // Create the filtering object
  pcl::ExtractIndices<PointT> extract;  //Saca puntos de la nube por indice

  int i = 0, nr_points = (int) input->size();

  *remainingCloud = *input;

  // While 30% of the original cloud is still there
  while (remainingCloud->size () > 0.1 * nr_points)
  { 
    std::stringstream ss;
    ss.str("");
    ss << "Plane_" << i;

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (remainingCloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }



    // Extract the inliers
    extract.setInputCloud (remainingCloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter(*pCloud);

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*fCloud);
    remainingCloud.swap(fCloud);
    i++;
  }
  std::cout << i <<" Planes Detected." << std::endl;
  return centroidCloud;
}
////////////////////////////////////////////////////////////////////////////////


int main(int argc, char **argv)
{
  // Get the data  
  PointCloud::Ptr cloud (new PointCloud);
  pcl::PCDReader reader;

  reader.read("map_Generalized.pcd", *cloud);
  
  pclVisualizer->setBackgroundColor(255,255,255);
  // Voxel Filter
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(0.05, 0.05, 0.05);
  sor.filter(*cloud);


  // Get Plane Indices 
  PointCloud::Ptr icpSource (new PointCloud);

  icpSource = extractIndices(cloud, 1000, 0.04);


  while (!pclVisualizer->wasStopped())
  {
    pclVisualizer->spinOnce (100);
    //boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    //std::this_thread::sleep_for(100ms);
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

    // float xn = coefficients->values[0]*coefficients->values[3];
    // float yn = coefficients->values[1]*coefficients->values[3];
    // float zn = coefficients->values[2]*coefficients->values[3];

    // PointT tmpPoint;

    // tmpPoint.x = -xn;
    // tmpPoint.y = -yn;
    // tmpPoint.z = -zn;

    std::cout << "Plane_" << i << '\n'; //": " <<"; "<< xn <<"; "<< yn <<"; "<< zn << '\n';

    // planeCoefsCloud->push_back(tmpPoint);

    
    // Extract the inliers
    extract.setInputCloud (original);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter(*pCloud);


    // // CREATE CLOUD WITH CENTROID //////////////////////////////////////////////
    // Eigen::Vector4f centroid;
    // PointT tmpPoint;

    // pcl::compute3DCentroid (*pCloud, centroid);
    
    // tmpPoint.x = centroid[0];
    // tmpPoint.y = centroid[1];
    // tmpPoint.z = centroid[2];

    // centroidCloud->push_back(tmpPoint);
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
    //std::cout << planeCoefsCloud->points << '\n';
    /*
    for (int z=0; z == planeCoefsCloud->size(); z++){
      std::cout << planeCoefsCloud->points[z] << '\n';
    }
    */
    // CloudColor sameColor (planeCoefsCloud, 0, 255, 240);
    // pclVisualizer->addPointCloud<PointT> (planeCoefsCloud, sameColor, "PointIndex");
    // pclVisualizer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "PointIndex");

    return fCloud; 
}




