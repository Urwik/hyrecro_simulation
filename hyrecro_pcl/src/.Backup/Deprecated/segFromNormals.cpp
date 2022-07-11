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
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

  // PCL FILTERS
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>

  // RANSAC
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

  // SEGMENTATION
#include <pcl/segmentation/sac_segmentation.h>

// Type Definitions
typedef pcl::PointCloud<pcl::PointXYZ> pointCloudXYZ;

// Global Variables
pointCloudXYZ::Ptr rosCloud;
pcl::visualization::PCLVisualizer::Ptr pclVisualizer (new pcl::visualization::PCLVisualizer ("PCL Visualizer"));
pcl::visualization::CloudViewer::Ptr cloudViewer;

// AVAILABLE FUNCTIONS /////////////////////////////////////////////////////////

// Filter the pointCloudXYZ by height (min,max)
pointCloudXYZ::Ptr filterHeight(const pointCloudXYZ::Ptr& original, float min, float max);

// Downsample pointCloudXYZ by a Voxel defined with x, y, z
pointCloudXYZ::Ptr downSample(const pointCloudXYZ::Ptr& original, float x, float y, float z);

// NormalEstimation
pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(pointCloudXYZ::Ptr original, float radiusNeigbour);

void SACsegFromNormals(pointCloudXYZ::Ptr original, pcl::PointCloud<pcl::Normal>::Ptr normals);

pcl::visualization::PCLVisualizer::Ptr normalsVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, 
                                                   pcl::PointCloud<pcl::Normal>::ConstPtr normals){

  pclVisualizer->removeAllPointClouds();
  pclVisualizer->addPointCloud<pcl::PointXYZ> (cloud, "Original_Cloud");
  pclVisualizer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Original_Cloud");
  pclVisualizer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
  return (pclVisualizer);
}
////////////////////////////////////////////////////////////////////////////////

void pcCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  ROS_INFO("Entra al Callback");
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input, pcl_pc2);
  pointCloudXYZ::Ptr temp_cloud(new pointCloudXYZ);
  pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);
  rosCloud = temp_cloud;
}

int main(int argc, char **argv)
{

  // ROS Stuff
  ros::init(argc, argv, "PC2_listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/os1/pointCloud", 1, pcCallback);
  ros::Rate loop_rate(1);

  pclVisualizer->setBackgroundColor (0, 0, 0);
  pclVisualizer->addCoordinateSystem (1.0);
  pclVisualizer->initCameraParameters ();


  while (ros::ok()){

    ros::spinOnce();
    pointCloudXYZ::Ptr filtered_cloud;

    // Filtering the cloud
    filtered_cloud = filterHeight(rosCloud, -1.0, 1.0);
    filtered_cloud = downSample(filtered_cloud, 0.03, 0.03, 0.03);

    pcl::PointCloud<pcl::Normal>::Ptr normals = estimateNormals(filtered_cloud, 0.1);
    pclVisualizer = normalsVis(filtered_cloud, normals);
    pclVisualizer->spinOnce(1000);

    SACsegFromNormals(filtered_cloud, normals);
    //Plot the cloud
    
    loop_rate.sleep();
  }

  return 0;
}

// FUNCTIONS DEFINITION

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

// Extract Normals
pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(pointCloudXYZ::Ptr original, float radiusNeigbour){

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (original);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch(radiusNeigbour);

  // Compute the features
  ne.compute (*cloud_normals);

  return cloud_normals;
}

// SAC Segmentation from Normals
void SACsegFromNormals(pointCloudXYZ::Ptr original, pcl::PointCloud<pcl::Normal>::Ptr normals){
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> ssfn;

  pointCloudXYZ::Ptr totalTmpCloud  (new pointCloudXYZ);
  pointCloudXYZ::Ptr tmpPlaneCloud  (new pointCloudXYZ);
  pointCloudXYZ::Ptr substractCloud (new pointCloudXYZ);
  pcl::PointCloud<pcl::Normal>::Ptr tmpNormalsCloud (new pcl::PointCloud<pcl::Normal>::Ptr);

  pcl::ModelCoefficients::Ptr modelCoefs (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  std::vector<pointCloudXYZ> planeVector;  

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::ExtractIndices<pcl::Normal> extractNormal;

  //Configure Segmentation
  ssfn.setInputCloud(original);
  ssfn.setInputNormals(normals);
  ssfn.setModelType(pcl::SACMODEL_NORMAL_PLANE);
  ssfn.setMethodType(pcl::SAC_RANSAC);
  ssfn.setDistanceThreshold(0.1);
  ssfn.setMaxIterations(1000);
  ssfn.setOptimizeCoefficients(true);
  ssfn.setNormalDistanceWeight(0.1);
  ssfn.setEpsAngle(0.1);
  
  //Segment the pointcloud
  ssfn.segment(*inliers, *modelCoefs);

  totalTmpCloud = original;
  tmpNormalsCloud = normals;
  int i = 0, nr_points = (int) original->size ();

  while (totalTmpCloud->size () > 0.1 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    ssfn.setInputCloud (totalTmpCloud);
    ssfn.setInputNormals(tmpNormalsCloud);

    ssfn.segment(*inliers, *modelCoefs);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Get a segment of the pointCloudXYZ
    extract.setInputCloud(totalTmpCloud);
    extract.setIndices(inliers);
    extractNormal.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*tmpPlaneCloud);


    if (!tmpPlaneCloud->empty())
      cloudViewer->showCloud(tmpPlaneCloud);
    else
      std::cout << "Cloud Empty" << '\n';

    //planeVector[i] = *tmpPlaneCloud;

    // Delete points extracted from the initial cloud
    extract.setNegative (true);
    extractNormal.setNegative(true);
    extract.filter(*substractCloud);
    extractNormal.filter(*tmpNormalsCloud);
    tmpNormalsCloud.swap(tmpNormalsCloud);
    totalTmpCloud.swap(substractCloud);

    i++;
  }
  //return(planeVector);
}