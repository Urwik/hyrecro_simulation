/*
  REGISTRA POR PARES 2 NUBES DE PUNTOS ESPECIFICADAS DENTRO DEL MAIN()
  CREA UN MAPA INTEGRANDO NUBES DE PUNTOS 2 A 2 DESDE ARCHIVOS  .pcd GUARDADOS.

  uso: rosrun hyrecro_pcl offline_mapping <path a donde estan la lista de archivos>  <ipo de icp>(punto_punto,punto_plano,plano_plano) <número_de_pointclouds> 
*/
// ROS 
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

// cpp
#include <iostream>
#include <algorithm>
#include <boost/filesystem.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry> 
#include <chrono>
#include <cmath>
#include <stdlib.h>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_representation.h>

  // PCL FILTERS
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/normal_space.h>

  // SEGMENTATION
#include <pcl/segmentation/sac_segmentation.h>

  // REGISTRATION
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_distance.h>


  // RANSAC
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

  // PCL VISUALIZATION
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>


//****************************************************************************//
// TYPE DEFINITIONS ////////////////////////////////////////////////////////////

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

typedef pcl::visualization::PointCloudColorHandlerCustom<PointT> CloudColor;


// ************************************************************************** //
using namespace boost::filesystem;

// ************************************************************************** //
// AVAILABLE FUNCTIONS /////////////////////////////////////////////////////////

// FEATURE EXTRACTION //

PointCloudWithNormals::Ptr  getCloudNormals(PointCloud::Ptr input, float radius);


// REGISTRATION //

Eigen::Matrix4f icpDefault(PointCloud::Ptr src, PointCloud::Ptr tgt);

Eigen::Matrix4f icpWithNormals(PointCloud::Ptr src, PointCloud::Ptr tgt);

Eigen::Matrix4f icpGeneralized(PointCloud::Ptr src, PointCloud::Ptr tgt);


////////////////////////////////////////////////////////////////////////////////

std::vector<double> fitScore;


int main(int argc, char **argv)
{
  //ROS Stuff
  ros::init(argc, argv, "offline_mapping");
  ros::NodeHandle n;
  ros::Publisher map_publisher = n.advertise<PointCloud>("map", 1, true);

  std::string folder_path = argv[1];
  std::string icp_method  = argv[2];
  long cloud_nmbr = strtol(argv[3], NULL, 10);

  std::cout << folder_path << std::endl;
  std::cout << icp_method << std::endl;

  ROS_INFO("Folder parth : %s", folder_path.c_str());
  ROS_INFO("ICP Method : %s", icp_method.c_str());

  // Get handlres for source and target cloud data /////////////////////////////
  PointCloud::Ptr source (new PointCloud);
  PointCloud::Ptr target (new PointCloud);

  PointCloud::Ptr alignedSource (new PointCloud);
  PointCloud::Ptr mapCloud (new PointCloud);

  std::stringstream ss;
  pcl::PCDReader reader;

  Eigen::Matrix4f tmpMth;
  Eigen::Matrix4f globalMth = Eigen::Matrix4f::Identity();

  auto start = std::chrono::high_resolution_clock::now();

  std::cout << "--- START MAPPING ---" << std::endl;

  for (int i = 0; i < cloud_nmbr-1; i++)
  {
    ss.str("");
    ss << folder_path << "/data" << i << ".pcd";
    reader.read(ss.str(), *target);
    
    ss.str("");
    ss << folder_path << "/data" << i+1 << ".pcd";
    reader.read(ss.str(), *source);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*source,*source, indices);
    pcl::removeNaNFromPointCloud(*target,*target, indices);


    // VOXEL FILTER //
    pcl::VoxelGrid<PointT> sor;
    sor.setLeafSize(0.1, 0.1, 0.1);

    sor.setInputCloud(target);
    sor.filter(*target);

    sor.setInputCloud(source);
    sor.filter(*source);


    // REGISTRATION
    if (icp_method == "point_point")
      tmpMth = icpDefault(source, target);
    else if (icp_method == "point_plane")
      tmpMth = icpWithNormals(source, target);
    else if (icp_method == "plane_plane")
      tmpMth = icpGeneralized(source, target);
    else
      ROS_ERROR("Second argument should be: point-point, point-plane or plane-plane");

    // TRANSFORMATION
    globalMth = globalMth * tmpMth;
    
    pcl::transformPointCloud (*source, *alignedSource, globalMth);

    *mapCloud += *alignedSource;
    sor.setInputCloud(mapCloud);
    sor.filter(*mapCloud);

    mapCloud->header.frame_id = "map";
    mapCloud->header.seq = i;
    pcl_conversions::toPCL(ros::Time::now(), mapCloud->header.stamp);

    map_publisher.publish(*mapCloud);
    ros::spinOnce();
  }
  
  std::cout << "--- MAP FINISHED ---" << std::endl;

  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop-start);
  std::cout << "Total time: " << duration.count() << std::endl;

  double sum = std::accumulate(fitScore.begin(), fitScore.end(), 0.0);
  double mean = sum / fitScore.size();
  std::cout << "Med Fit Score: " << mean << std::endl;
  double max = *std::max_element(fitScore.begin(), fitScore.end());
  std::cout << "Max error: " << max << std::endl;

  pcl::PCDWriter writer;
  ss.str("");
  ss << folder_path << "/" << icp_method << "_map.pcd";
  writer.write<PointT>(ss.str(), *mapCloud, false);

  while(n.ok()){}

  return 0;
}


// ************************************************************************** //
// FUNCTIONS DEFINITIONS ///////////////////////////////////////////////////////


// FEATURE EXTRACTION //

PointCloudWithNormals::Ptr getCloudNormals(PointCloud::Ptr input, float radius){

  // ESTIMATE NORMALS //////////////////////////////////////////////////////////
  pcl::NormalEstimation<PointT, PointNormalT> ne;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

  PointCloudWithNormals::Ptr cloudNormals (new PointCloudWithNormals);  

  ne.setInputCloud(input);
  ne.setSearchMethod (tree);
  ne.setKSearch(5);
  //ne.setRadiusSearch(radius);
  ne.compute(*cloudNormals);

  pcl::copyPointCloud(*input, *cloudNormals);

  return cloudNormals;
}

// REGISTRATION //

Eigen::Matrix4f icpDefault(PointCloud::Ptr src, PointCloud::Ptr tgt){
  pcl::IterativeClosestPoint<PointT, PointT> icp;

  PointCloud::Ptr result (new PointCloud);

  icp.setInputSource(src);
  icp.setInputTarget(tgt);
  icp.align(*result);

  Eigen::Matrix4f mth;
  mth = icp.getFinalTransformation();

  fitScore.push_back(icp.getFitnessScore());

  return mth;
}

Eigen::Matrix4f icpWithNormals(PointCloud::Ptr src, PointCloud::Ptr tgt){
  PointCloudWithNormals::Ptr srcWithNormals (new PointCloudWithNormals);
  PointCloudWithNormals::Ptr tgtWithNormals (new PointCloudWithNormals);

  pcl::IterativeClosestPointWithNormals<PointNormalT, PointNormalT> icpWN;

  PointCloudWithNormals tmpOutputCloud;

  srcWithNormals = getCloudNormals(src, 0.05);
  tgtWithNormals = getCloudNormals(tgt, 0.05);

  icpWN.setInputSource(srcWithNormals);
  icpWN.setInputTarget(tgtWithNormals);

  icpWN.align(tmpOutputCloud);

  Eigen::Matrix4f mth;
  mth = icpWN.getFinalTransformation();
  fitScore.push_back(icpWN.getFitnessScore());

  return mth;
}

Eigen::Matrix4f icpGeneralized(PointCloud::Ptr src, PointCloud::Ptr tgt){

  pcl::GeneralizedIterativeClosestPoint<PointT, PointT> genIcp;

  PointCloud::Ptr result (new PointCloud);

  genIcp.setInputSource(src);
  genIcp.setInputTarget(tgt);
  
  genIcp.align(*result);

  Eigen::Matrix4f mth;
  mth = genIcp.getFinalTransformation();
  fitScore.push_back(genIcp.getFitnessScore());
  
  return mth;
}