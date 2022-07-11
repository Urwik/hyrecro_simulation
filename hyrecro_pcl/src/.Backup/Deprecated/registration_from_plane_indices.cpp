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

  // REGISTRATION
#include <pcl/registration/icp.h>

  // RANSAC
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

  // PCL VISUALIZATION
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>





//****************************************************************************//
// Type Definitions ////////////////////////////////////////////////////////////
typedef pcl::PointCloud<pcl::PointXYZ> pointCloudXYZ;

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

// Global Variables 

    // Visualizers
pcl::visualization::PCLVisualizer::Ptr pclVisualizer (new pcl::visualization::PCLVisualizer ("PCL Visualizer"));
pcl::visualization::CloudViewer cloudViewer ("Simple Cloud Viewer");
    
    // Clouds
pointCloudXYZ::Ptr inputCloud (new pointCloudXYZ);
pointCloudXYZ::Ptr mapCloud (new pointCloudXYZ);
    
    // Structs
struct IndxCoefs{pcl::PointIndices::Ptr inliers; pcl::ModelCoefficients::Ptr coefs;};

int cuenta=0;

// ************************************************************************** //
// ************************************************************************** //





// ************************************************************************** //
// AVAILABLE FUNCTIONS /////////////////////////////////////////////////////////

// Show a pointCloudXYZ
void plotCloud(const pointCloudXYZ::Ptr& cloud, std::string cloudName);

// Filter the pointCloudXYZ by height (min,max)
pointCloudXYZ::Ptr filterHeight(const pointCloudXYZ::Ptr& original, float min, float max);

// Downsample pointCloudXYZ by a Voxel defined with x, y, z
pointCloudXYZ::Ptr downSample(const pointCloudXYZ::Ptr& original, float x, float y, float z);

// Outlier Removal
pointCloudXYZ::Ptr outlierRemoval(const pointCloudXYZ::Ptr& original, int neighbors, float stdev);

// Project inliers to a plane
pointCloudXYZ::Ptr projectToPlane(const pointCloudXYZ::Ptr& original, float x, float y, float z, float d);

// Extract indices
pointCloudXYZ::Ptr extractIndices(pointCloudXYZ::Ptr original, int maxIterations, float threshold);

// Planar Segmentation
void planarSegmentation(pointCloudXYZ::Ptr original, int maxIterations, float threshold);

// Show a cloud with specific color
pcl::visualization::PCLVisualizer::Ptr customColourVis (pointCloudXYZ::ConstPtr cloud, std::string cloudName, pcl::RGB color)
{
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, (int) color.r, (int) color.g, (int) color.b);
  pclVisualizer->addPointCloud<pcl::PointXYZ> (cloud, single_color, cloudName);
  pclVisualizer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloudName);
  return (pclVisualizer);
}

// Registration
pointCloudXYZ regClouds(pointCloudXYZ::Ptr source, pointCloudXYZ::Ptr target);
// ************************************************************************** //
// ************************************************************************** //





// ************************************************************************** //
// MAIN FUNCTIONS //////////////////////////////////////////////////////////////


// CALLBACK //
void pcCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input, pcl_pc2);
  pointCloudXYZ::Ptr temp_cloud (new pointCloudXYZ);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

  inputCloud = temp_cloud;
}


// MAIN //
int main(int argc, char **argv)
{
  ros::init(argc, argv, "PC2_listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/os1/pointCloud", 1, pcCallback);
  ros::Rate rate(1);

  pclVisualizer->setBackgroundColor (0, 0, 0);
  pclVisualizer->addCoordinateSystem (1.0);
  pclVisualizer->initCameraParameters ();

  while(ros::ok()){
    ros::spinOnce();
    pclVisualizer->removeAllPointClouds();
    pointCloudXYZ::Ptr filtered_cloud = filterHeight(inputCloud, -0.50, 0.50);
    pointCloudXYZ::Ptr ds_cloud = downSample(filtered_cloud, 0.02f, 0.02f, 0.02f);
    //pointCloudXYZ::Ptr indexCloud = extractIndices(ds_cloud, 1000, 0.01);


    if (mapCloud->empty()){ //The first time
      *mapCloud = *ds_cloud;
    }
    else{
      pointCloudXYZ tmpCloud = regClouds(ds_cloud, mapCloud);
      *mapCloud += tmpCloud;
      mapCloud = downSample(mapCloud, 0.07f, 0.07f, 0.07f);
      cloudViewer.showCloud(mapCloud);  
    }

    rate.sleep();
  }
  return 0;
}


// ************************************************************************** //
// ************************************************************************** //








// ************************************************************************** //
// FUNCTIONS DEFINITIONS ///////////////////////////////////////////////////////

// Show a pointCloudXYZ
void plotCloud(const pointCloudXYZ::Ptr& cloud, std::string cloudName){
  cloudViewer.showCloud(cloud, cloudName);
}

// Filter the pointCloudXYZ by height (min,max)
pointCloudXYZ::Ptr filterHeight(const pointCloudXYZ::Ptr& original, float min, float max){
  pointCloudXYZ::Ptr filtered_cloud (new pointCloudXYZ);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(original);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(min, max);
  pass.filter(*filtered_cloud);

  return filtered_cloud;
}

// Downsample pointCloudXYZ by a Voxel defined with x, y, z
pointCloudXYZ::Ptr downSample(const pointCloudXYZ::Ptr& original, float x, float y, float z){
  pointCloudXYZ::Ptr ds_cloud (new pointCloudXYZ);
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(original);
  sor.setLeafSize(x, y, z);
  sor.filter(*ds_cloud);

  return ds_cloud;
}

// Extract indices
pointCloudXYZ::Ptr extractIndices(pointCloudXYZ::Ptr original, int maxIterations, float threshold){
  pointCloudXYZ::Ptr pCloud (new pointCloudXYZ);
  pointCloudXYZ::Ptr fCloud (new pointCloudXYZ);
  pcl::RGB rgb;
  

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (maxIterations);
  seg.setDistanceThreshold (threshold);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  int i = 0, nr_points = (int) original->size ();
  // While 30% of the original cloud is still there
  while (original->size () > 0.1 * nr_points)
  {

    std::stringstream ss;
    ss.str("");
    ss << "Cloud_" << i;

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (original);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

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
    i++;
  }
    return fCloud; 
}

// Planar Segmentation
void planarSegmentation(pointCloudXYZ::Ptr original, int maxIterations, float threshold){

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (maxIterations);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (original);
  
  seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return; 
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  for (std::size_t i = 0; i < inliers->indices.size (); ++i)
  for (const auto& idx: inliers->indices)
    std::cerr << idx << "    " << original->points[idx].x << " "
                               << original->points[idx].y << " "
                               << original->points[idx].z << std::endl;

  return;
}

// Registration
pointCloudXYZ regClouds(pointCloudXYZ::Ptr source, pointCloudXYZ::Ptr target){

  if (target->empty())
    ROS_ERROR("Target Cloud its empty");

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  pointCloudXYZ tmpOutputCloud;
  icp.setInputSource(source);
  icp.setInputTarget(target);

  icp.align(tmpOutputCloud);

  std::cout << "Has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;

  if (icp.getFitnessScore() < 0.001)
    return tmpOutputCloud;
  else
    return *target;

}  


// ************************************************************************** //