/*
  REGISTRA POR PARES 2 NUBES DE PUNTOS ESPECIFICADAS DENTRO DEL MAIN()
*/
// cpp
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry> 

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
#include <pcl/registration/gicp.h>


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
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointT> CloudColor;




// ************************************************************************** //


// Global Variables ////////////////////////////////////////////////////////////
    // Visualizers


int cuenta=0;

// ************************************************************************** //
// AVAILABLE FUNCTIONS /////////////////////////////////////////////////////////

// Filter the PointCloud by height (min,max)
PointCloud::Ptr filterHeight(const PointCloud::Ptr& original, float min, float max);

// Downsample PointCloud by a Voxel defined with x, y, z
PointCloud::Ptr downSample(const PointCloud::Ptr& original, float x, float y, float z);

// Extract indices
PointCloud::Ptr extractIndices(PointCloud::Ptr original, int maxIterations, float threshold);

// Register Clouds
Eigen::Matrix4f regClouds(PointCloud::Ptr source, PointCloud::Ptr target);

Eigen::Matrix4f icpNormals(PointCloud::Ptr source, PointCloud::Ptr target);

Eigen::Matrix4f estimMth(PointCloud::Ptr source, PointCloud::Ptr target);

Eigen::Matrix4f genICP(PointCloud::Ptr source, PointCloud::Ptr target);
////////////////////////////////////////////////////////////////////////////////


int main(int argc, char **argv)
{
  // Get the data  
  PointCloud::Ptr source (new PointCloud);
  PointCloud::Ptr target (new PointCloud);

  pcl::PCDReader reader;

  reader.read("data1_sampled.pcd", *target);
  reader.read("data2_sampled.pcd", *source);

  // Voxel Filter
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud(target);
  sor.setLeafSize(0.1, 0.1, 0.1);
  sor.filter(*target);

  sor.setInputCloud(source);
  sor.setLeafSize(0.1, 0.1, 0.1);
  sor.filter(*source);

  // PointCloud::Ptr resultado (new PointCloud);
  // resultado = genICP(source, target);

  
  // Get Plane Indices 
  PointCloud::Ptr icpSource (new PointCloud);
  PointCloud::Ptr icpTarget (new PointCloud);

  icpSource = extractIndices(source, 1000, 0.01);
  icpTarget = extractIndices(target, 1000, 0.01);

  // Get Transformation Matrix between clouds
  Eigen::Matrix4f estimatedMth;
  Eigen::Matrix4f mth;

  // REGISTRATION EN 2 FASES
  estimatedMth = estimMth(icpSource, icpTarget);  // ICP only plane indx
  PointCloud::Ptr estimatedSource (new PointCloud);
  pcl::transformPointCloud (*source, *estimatedSource, estimatedMth);

  sor.setInputCloud(target);
  sor.setLeafSize(0.3, 0.3, 0.3);
  sor.filter(*target);

  sor.setInputCloud(estimatedSource);
  sor.setLeafSize(0.3, 0.3, 0.3);
  sor.filter(*estimatedSource);


  mth = regClouds(estimatedSource, target); // ICP 
  
  //mth = regClouds(source, target);        // ICP 
  //mth = genICP(source, target);           //GENERALIZED ICP

  // Transform original clouds using the transformation matrix
  PointCloud::Ptr tfSource (new PointCloud);
  pcl::transformPointCloud (*source, *tfSource, mth);


  // Visualize clouds //////////////////////////////////////////////////////////
  
  CloudColor red   (source, 255, 0, 0);
  CloudColor green (target, 0, 255, 0);

  CloudColor red2  (tfSource, 255, 0, 0);

  CloudColor blue (icpSource, 0, 0, 255);
  CloudColor ye   (icpTarget, 255, 255, 0);
  CloudColor prpl (estimatedSource, 230, 0, 255);
  //std::cout << icpTarget->size() << '\n';
  
  
  pcl::visualization::PCLVisualizer::Ptr visOriginal (new pcl::visualization::PCLVisualizer ("Original Visualizer"));
  pcl::visualization::PCLVisualizer::Ptr visTF (new pcl::visualization::PCLVisualizer ("Transformed Visualizer"));
  
  visOriginal->addPointCloud<PointT> (source,    red,   "SourceCloud");
  visOriginal->addPointCloud<PointT> (target,    green, "TargetCloud");
  visOriginal->addPointCloud<PointT> (icpSource, blue,  "icpSourceCloud");
  visOriginal->addPointCloud<PointT> (icpTarget, ye,    "icpTargetCloud");


  visOriginal->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "SourceCloud");
  visOriginal->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "TargetCloud");
  visOriginal->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "icpSourceCloud");
  visOriginal->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "icpTargetCloud");

  visTF->addPointCloud<PointT> (target, green, "TargetCloud");
  //visTF->addPointCloud<PointT> (tfSource, red2, "tfSourceCloud");
  visTF->addPointCloud<PointT> (estimatedSource, prpl, "EstimatedSource");

  visTF->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "TargetCloud");
  //visTF->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "tfSourceCloud");
  visTF->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "EstimatedSource");


  while (!visOriginal->wasStopped() && !visTF->wasStopped()  )
  {
    visOriginal->spinOnce (100);
    visTF->spinOnce (100);
    //boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    //std::this_thread::sleep_for(100ms);
  }

  return 0;
}


// ************************************************************************** //
// FUNCTIONS DEFINITIONS ///////////////////////////////////////////////////////

// Filter the PointCloud by height (min,max)
PointCloud::Ptr filterHeight(const PointCloud::Ptr& original, float min, float max){
  PointCloud::Ptr filtered_cloud (new PointCloud);
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(original);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(min, max);
  pass.filter(*filtered_cloud);

  return filtered_cloud;
}

// Downsample PointCloud by a Voxel defined with x, y, z
PointCloud::Ptr downSample(const PointCloud::Ptr& original, float x, float y, float z){
  PointCloud::Ptr ds_cloud (new PointCloud);
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud(original);
  sor.setLeafSize(x, y, z);
  sor.filter(*ds_cloud);

  return ds_cloud;
}

// Extract indices
PointCloud::Ptr extractIndices(PointCloud::Ptr input, int maxIterations, float threshold){
  PointCloud::Ptr planeCoefsCloud (new PointCloud);
  PointCloud::Ptr centroidCloud (new PointCloud);
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
  while (remainingCloud->size () > 0.3 * nr_points)
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


/*
    Eigen::Vector4f centroid;

    pcl::compute3DCentroid (*pCloud, centroid);

    PointT tmpPoint2;

    tmpPoint2.x = centroid[0];
    tmpPoint2.y = centroid[1];
    tmpPoint2.z = centroid[2];
    
    centroidCloud->push_back(tmpPoint2);
*/

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

  //return centroidCloud; 
  return planeCoefsCloud; 
}

// Registration
Eigen::Matrix4f regClouds(PointCloud::Ptr source, PointCloud::Ptr target){

  pcl::IterativeClosestPoint<PointT, PointT> icp;

  PointCloud tmpOutputCloud;
  //icp.setTransformationEstimation();
  //icp.setMaxCorrespondenceDistance(0.01);
  icp.setInputSource(source);
  icp.setInputTarget(target);
  icp.setMaximumIterations(1000);
  //icp.setTransformationEpsilon(1e-20);
  //icp.setEuclideanFitnessEpsilon(0.001);

  icp.align(tmpOutputCloud);

  Eigen::Matrix4f mth;
  mth = icp.getFinalTransformation();

  std::cout << "Has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  //std::cout << icp.getFinalTransformation() << std::endl;

  return mth;
}


Eigen::Matrix4f estimMth(PointCloud::Ptr source, PointCloud::Ptr target){
  
  pcl::IterativeClosestPoint<PointT, PointT> icp;

  PointCloud tmpOutputCloud;
  PointCloud::Ptr aproxedCloud (new PointCloud);
  //icp.setTransformationEstimation();
  //icp.setMaxCorrespondenceDistance(0.01);
  icp.setInputSource(source);
  icp.setInputTarget(target);
  icp.setMaximumIterations(1000);
  icp.setTransformationEpsilon(1e-20);
  icp.setEuclideanFitnessEpsilon(0.00001);

  icp.align(tmpOutputCloud);

  Eigen::Matrix4f mth;
  mth = icp.getFinalTransformation();

  pcl::transformPointCloud (*source, *aproxedCloud, mth);

  std::cout << "Has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  //std::cout << icp.getFinalTransformation() << std::endl;

  return mth;
}


// Generalized ICP
Eigen::Matrix4f genICP(PointCloud::Ptr source, PointCloud::Ptr target){

  pcl::GeneralizedIterativeClosestPoint<PointT, PointT> genIcp;

  PointCloud::Ptr result (new PointCloud);

  genIcp.setInputSource(source);
  genIcp.setInputTarget(target);
  
  genIcp.align(*result);

  Eigen::Matrix4f mth;
  mth = genIcp.getFinalTransformation();

  std::cout << "Has converged:" << genIcp.hasConverged() << " score: " <<
  genIcp.getFitnessScore() << std::endl;

  return mth;
}