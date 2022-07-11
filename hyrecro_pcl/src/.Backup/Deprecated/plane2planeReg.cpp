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
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudNormals;


// ************************************************************************** //
// Global Variables ////////////////////////////////////////////////////////////
    // Visualizers
//pcl::visualization::PCLVisualizer::Ptr pclVisualizer (new pcl::visualization::PCLVisualizer ("PCL Visualizer"));
pcl::visualization::CloudViewer cloudViewer ("Cloud Viewer"); // (new pcl::visualization::CloudViewer);  
    // Clouds
PointCloud::Ptr rosCloud (new PointCloud);


int cuenta=0;

// ************************************************************************** //
// AVAILABLE FUNCTIONS /////////////////////////////////////////////////////////

// Filter the PointCloud by height (min,max)
PointCloud::Ptr filterHeight(const PointCloud::Ptr& original, float min, float max);

// Downsample PointCloud by a Voxel defined with x, y, z
PointCloud::Ptr downSample(const PointCloud::Ptr& original, float x, float y, float z);

// Extract indices
PointCloud::Ptr extractIndices(PointCloud::Ptr original, int maxIterations, float threshold);

void saveToFile(PointCloud::Ptr cloud, std::string fileName);


bool robotMoved(tf::StampedTransform prevTransform);

// Show a cloud with specific color
/*
pcl::visualization::PCLVisualizer::Ptr customColourVis (PointCloud::ConstPtr cloud, std::string cloudName, pcl::RGB color)
{
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, (int) color.r, (int) color.g, (int) color.b);
  pclVisualizer->addPointCloud<pcl::PointXYZ> (cloud, single_color, cloudName);
  pclVisualizer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloudName);
  return (pclVisualizer);
}
*/
////////////////////////////////////////////////////////////////////////////////


void pcCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input, pcl_pc2);
  PointCloud::Ptr temp_cloud (new PointCloud);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

  *rosCloud = *temp_cloud;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "PC2_listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/os1/pointCloud", 1, pcCallback);
  ros::Publisher pubTarget = n.advertise<sensor_msgs::PointCloud2>("targetCloud", 1);
  ros::Publisher pubSource = n.advertise<sensor_msgs::PointCloud2>("sourceCloud", 1);
  ros::Rate rate(1);

  tf::StampedTransform prevTransform;
  tf::TransformListener::lookupTransform("/ouser_sensor","/base_link", 
                                          ros::Time(0), prevTransform);

  PointCloud::Ptr source (new PointCloud);
  PointCloud::Ptr target (new PointCloud);

  //pclVisualizer->setBackgroundColor (0, 0, 0);
  //pclVisualizer->addCoordinateSystem (1.0);
  //pclVisualizer->initCameraParameters ();

  int iteration = 1;
  while(ros::ok()){
    ros::spinOnce();

    if(rosCloud->empty()){
      continue;
      ROS_INFO("Empty Cloud in Iteration: %d", iteration);
    }

    else if (iteration == 1){
      PointCloud::Ptr filtered_cloud = filterHeight(rosCloud, -0.50, 0.50);
      PointCloud::Ptr ds_cloud = downSample(filtered_cloud, 0.02f, 0.02f, 0.02f);
      
      *target = *extractIndices(ds_cloud, 100, 0.01);
    }


    if robotMoved(prevTransform){

      PointCloud::Ptr filtered_cloud = filterHeight(rosCloud, -0.50, 0.50);
      PointCloud::Ptr ds_cloud = downSample(filtered_cloud, 0.02f, 0.02f, 0.02f);
      
      *source = *extractIndices(ds_cloud, 100, 0.01);

      pcl::IterativeClosestPoint<PointT, PointT> icp;

      PointCloud tmpOutputCloud;
 
      

      icp.setInputSource(source);
      icp.setInputTarget(target);

      icp.align(tmpOutputCloud);

      Eigen::Matrix4f mth = icp.getFinalTransformation ();

      std::cout << "Has converged:" << icp.hasConverged() << " score: " <<
      icp.getFitnessScore() << std::endl;
      std::cout << icp.getFinalTransformation() << std::endl;

    }

    //pclVisualizer->removeAllPointClouds();
    ROS_INFO("Loop Iteraction: %d", iteration);
    iteration++;
    rate.sleep();
  }
  return 0;
}


// ************************************************************************** //
// FUNCTIONS DEFINITIONS ///////////////////////////////////////////////////////





bool robotMoved(tf::StampedTransform prevTransform){
  tf::TransformListener listener;
  tf::StampedTransform transform;
  listener.lookupTransform("/ouser_sensor","/base_link", ros::Time(0), transform);

  if (prevTransform == transform)
    return false;
  else
    return true;
}

void saveToFile(PointCloud::Ptr cloud, std::string fileName){
  pcl::PCDWriter writer;
  std::stringstream ss;
  ss.str("");
  ss <<  fileName << ".pcd";
  writer.write<PointT>(ss.str(), *cloud, false);
}


// Filter the PointCloud by height (min,max)
PointCloud::Ptr filterHeight(const PointCloud::Ptr& input, float min, float max){
  PointCloud::Ptr filtered_cloud (new PointCloud);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(input);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(min, max);
  pass.filter(*filtered_cloud);

  return filtered_cloud;
}

// Downsample PointCloud by a Voxel defined with x, y, z
PointCloud::Ptr downSample(const PointCloud::Ptr& input, float x, float y, float z){
  PointCloud::Ptr ds_cloud (new PointCloud);
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(input);
  sor.setLeafSize(x, y, z);
  sor.filter(*ds_cloud);

  return ds_cloud;
}

// Extract indices
PointCloud::Ptr extractIndices(PointCloud::Ptr input, int maxIterations, float threshold){
  PointCloud::Ptr planeCoefsCloud (new PointCloud);
  PointCloud::Ptr remainingCloud (new PointCloud);
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
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(maxIterations);
  seg.setDistanceThreshold(threshold);

  // Create the filtering object
  pcl::ExtractIndices<PointT> extract;

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

    float xn = coefficients->values[0];
    float yn = coefficients->values[1];
    float zn = coefficients->values[2];

    PointT tmpPoint;

    tmpPoint.x = xn;
    tmpPoint.y = yn;
    tmpPoint.z = zn;

    planeCoefsCloud->push_back(tmpPoint);

    //cloudViewer.showCloud(planeCoefsCloud);
    //std::cout << "Points In the Cloud: " << planeCoefsCloud->points.size() << std::endl;
    // Extract the inliers
    extract.setInputCloud (remainingCloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter(*pCloud);

    //**************************************
    // Generate unique colour
    rgb = pcl::GlasbeyLUT::at(i);

    //plotCloud(pCloud, ss.str());
    //customColourVis(pCloud, ss.str(), rgb);
    
    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*fCloud);
    remainingCloud.swap(fCloud);
    i++;
  }

  //std::cout << "Points In the Cloud: " << planeCoefsCloud->points.size() << std::endl;

  return planeCoefsCloud; 
}