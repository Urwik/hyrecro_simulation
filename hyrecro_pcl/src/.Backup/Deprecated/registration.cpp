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
#include <pcl/filters/normal_space.h>

  // SEGMENTATION
#include <pcl/segmentation/sac_segmentation.h>

  // REGISTRATION
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_distance.h>


  // RANSAC
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

  // PCL VISUALIZATION
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>


//****************************************************************************//
// TYPE DEFINITIONS ////////////////////////////////////////////////////////////

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

typedef pcl::PointCloud<pcl::PointXYZI> PointCloudIntensity;

typedef pcl::visualization::PointCloudColorHandlerCustom<PointT> CloudColor;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNormalT> NormalCloudColor;
typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> IntensityCloudColor;


// ************************************************************************** //


// ************************************************************************** //
// AVAILABLE FUNCTIONS /////////////////////////////////////////////////////////

// FEATURE EXTRACTION //

PointCloud::Ptr             getPlaneCoefsCloud(PointCloud::Ptr original, int maxIterations, float threshold);

PointCloud::Ptr             getCentroidCloud(PointCloud::Ptr input, int maxIterations, float threshold);

PointCloudIntensity::Ptr    getIntensityPlaneCoefsCloud(PointCloud::Ptr input, int maxIterations, float threshold);

PointCloudWithNormals::Ptr  getCentroidNormalPlaneCloud(PointCloud::Ptr original, int maxIterations, float threshold);

pcl::PointCloud<pcl::Normal>::Ptr getCloudNormals(PointCloud::Ptr input, float radius);


// REGISTRATION //

Eigen::Matrix4f icpDefault(PointCloud::Ptr source, PointCloud::Ptr target);

Eigen::Matrix4f icpGeneralized(PointCloud::Ptr source, PointCloud::Ptr target);

Eigen::Matrix4f icpWithNormals(PointCloudWithNormals::Ptr source, PointCloudWithNormals::Ptr target);

Eigen::Matrix4f icpWithIntensity(PointCloudIntensity::Ptr source, PointCloudIntensity::Ptr target);

Eigen::Matrix4f estimMth(PointCloud::Ptr source, PointCloud::Ptr target);


////////////////////////////////////////////////////////////////////////////////


int main(int argc, char **argv)
{

  // Get handlres for source and target cloud data /////////////////////////////
  PointCloud::Ptr source (new PointCloud);
  PointCloud::Ptr target (new PointCloud);

  pcl::PCDReader reader;

  reader.read("data50.pcd", *target);
  reader.read("data51.pcd", *source);
  
  // VOXEL FILTER //
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud(target);
  sor.setLeafSize(0.1, 0.1, 0.1);
  sor.filter(*target);

  sor.setInputCloud(source);
  //sor.setLeafSize(0.1, 0.1, 0.1);
  sor.filter(*source);


  // // PLANE COEFS CLOUD //
  // PointCloud::Ptr featureCloudSource (new PointCloud);
  // PointCloud::Ptr featureCloudTarget (new PointCloud);

  // std::cout << "Source Cloud: ";
  // featureCloudSource = getPlaneCoefsCloud(source, 1000, 0.02);
  // std::cout << "Target Cloud: ";
  // featureCloudTarget = getPlaneCoefsCloud(target, 1000, 0.02);

  // // CENTROID CLOUD //
  // PointCloud::Ptr featureCloudSource (new PointCloud);
  // PointCloud::Ptr featureCloudTarget (new PointCloud);

  // //std::cout << "Source Cloud: ";
  // featureCloudSource = getCentroidCloud(source, 1000, 0.02);
  // //std::cout << "Target Cloud: ";
  // featureCloudTarget = getCentroidCloud(target, 1000, 0.02);

  // INTENSITY PLANE COEFS CLOUD //
  // PointCloudIntensity::Ptr featureCloudSource (new PointCloudIntensity);
  // PointCloudIntensity::Ptr featureCloudTarget (new PointCloudIntensity);

  // featureCloudSource = getIntensityPlaneCoefsCloud(source, 1000, 0.01);
  // featureCloudTarget = getIntensityPlaneCoefsCloud(target, 1000, 0.01);


  // INTENSITY PLANE COEFS CLOUD //
  // PointCloudWithNormals::Ptr featureCloudSource (new PointCloudWithNormals);
  // PointCloudWithNormals::Ptr featureCloudTarget (new PointCloudWithNormals);

  // featureCloudSource = getCentroidNormalPlaneCloud(source, 1000, 0.01);
  // featureCloudTarget = getCentroidNormalPlaneCloud(target, 1000, 0.01);

/*
  // VOXEL FILTER //
  sor.setInputCloud(featureCloudSource);
  sor.setLeafSize(0.5, 0.5, 0.5);
  sor.filter(*featureCloudSource);

  sor.setInputCloud(featureCloudTarget);
  sor.setLeafSize(0.5, 0.5, 0.5);
  sor.filter(*featureCloudTarget);
*/

  // REGISTRATION //
  Eigen::Matrix4f estimatedMth;
  Eigen::Matrix4f mth;


  // ICP DEFAULT // 
  mth = icpDefault(source, target);


  // GENERALIZED ICP //       
  //mth = icpGeneralized(source, target);


  // // ICP WITH PLANE INDEX
  // mth = icpDefault(featureCloudSource, featureCloudTarget);
  // std::cout << mth << '\n';


  // ICP WITH INTENSITY
  //mth = icpWithIntensity(featureCloudSource, featureCloudTarget);

  // ICP WITH CENTROID AND PLANE INDEX
  //mth = icpWithNormals(featureCloudSource, featureCloudTarget);


  // REGISTRATION EN 2 FASES
  //estimatedMth = estimMth(featureCloudSource, featureCloudTarget);  // Estimate T with plane indx
  //PointCloud::Ptr estimatedSource (new PointCloud);
  //pcl::transformPointCloud (*source, *estimatedSource, estimatedMth);
  //mth = icpDefault(estimatedSource, target);       


  // TRANSFORMATION //

  PointCloud::Ptr tfSource (new PointCloud);
  pcl::transformPointCloud (*source, *tfSource, mth);


  // VISUALIZATION // 
  
  CloudColor red   (source,   255,    0,    0);
  CloudColor green (target,     0,  255,    0);
  CloudColor red2  (tfSource, 255,    0,    0);

  // CloudColor prpl  (featureCloudSource,   230, 0, 255);
  // CloudColor ye    (featureCloudTarget, 255, 255, 0);

  // CloudColor ye      (featureCloudSource,   255,    255,    0);
  // CloudColor blue    (featureCloudTarget,  0,  0,    255);

  //IntensityCloudColor blue (icpSource, 0, 0, 255);
  //IntensityCloudColor ye   (icpTarget, 255, 255, 0);
  //CloudColor prpl (estimatedSource, 230, 0, 255);
  
  
  pcl::visualization::PCLVisualizer::Ptr visOriginal (new pcl::visualization::PCLVisualizer ("Original Visualizer"));
  pcl::visualization::PCLVisualizer::Ptr visTF (new pcl::visualization::PCLVisualizer ("Transformed Visualizer"));
  
  // ORIGINAL //

  visOriginal->addPointCloud<PointT> (source,    red,   "SourceCloud");
  visOriginal->addPointCloud<PointT> (target,    green, "TargetCloud");
  // visOriginal->addPointCloud<PointT> (featureCloudSource, ye,  "featureCloudSource");
  // visOriginal->addPointCloud<PointT> (featureCloudTarget, blue,    "featureCloudTarget");

  visOriginal->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "SourceCloud");
  visOriginal->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "TargetCloud");
  //visOriginal->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 12, "featureCloudSource");
  //visOriginal->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 12, "featureCloudTarget");


  // TRANSFORMED //

  visTF->addPointCloud<PointT> (target, green, "TargetCloud");
  visTF->addPointCloud<PointT> (tfSource, red, "tfSourceCloud");

  visTF->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "TargetCloud");
  visTF->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "tfSourceCloud");


  while (!visOriginal->wasStopped() && !visTF->wasStopped())
  {
    visOriginal->spinOnce (100);
    visTF->spinOnce (100);
    //normalVis.spinOnce(100);
  }

  return 0;
}


// ************************************************************************** //
// FUNCTIONS DEFINITIONS ///////////////////////////////////////////////////////


// FEATURE EXTRACTION //

PointCloud::Ptr getPlaneCoefsCloud(PointCloud::Ptr input, int maxIterations, float threshold){
  PointCloud::Ptr planeCoefsCloud (new PointCloud);
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

    PointT tmpPoint;

    tmpPoint.x = -1 * coefficients->values[0]*coefficients->values[3];
    tmpPoint.y = -1 * coefficients->values[1]*coefficients->values[3];
    tmpPoint.z = -1 * coefficients->values[2]*coefficients->values[3];

    planeCoefsCloud->push_back(tmpPoint);

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
  return planeCoefsCloud;
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

    // CREATE CLOUD WITH CENTROID AND PLANE COEFFS /////////////////////////////
    Eigen::Vector4f centroid;
    PointT tmpPoint;

    pcl::compute3DCentroid (*pCloud, centroid);
    
    tmpPoint.x = centroid[0];
    tmpPoint.y = centroid[1];
    tmpPoint.z = centroid[2];

    centroidCloud->push_back(tmpPoint);

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

PointCloudIntensity::Ptr getIntensityPlaneCoefsCloud(PointCloud::Ptr input, int maxIterations, float threshold){
  
  PointCloudIntensity::Ptr intensityPlaneCoefsCloud (new PointCloudIntensity);
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

    pcl::PointXYZI tmpPoint;

    tmpPoint.x = coefficients->values[0];
    tmpPoint.y = coefficients->values[1];
    tmpPoint.z = coefficients->values[2];
    tmpPoint.intensity = coefficients->values[3];
     
    intensityPlaneCoefsCloud->push_back(tmpPoint);

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

  return intensityPlaneCoefsCloud;
}

PointCloudWithNormals::Ptr getCentroidNormalPlaneCloud(PointCloud::Ptr input, int maxIterations, float threshold){
  PointCloudWithNormals::Ptr centroidNormalCloud (new PointCloudWithNormals);
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

    // Extract the inliers
    extract.setInputCloud (remainingCloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter(*pCloud);

    // CREATE CLOUD WITH CENTROID AND PLANE COEFFS /////////////////////////////
    Eigen::Vector4f centroid;
    PointNormalT tmpN_Point;

    pcl::compute3DCentroid (*pCloud, centroid);
    
    tmpN_Point.x = centroid[0];
    tmpN_Point.y = centroid[1];
    tmpN_Point.z = centroid[2];
    tmpN_Point.normal_x = coefficients->values[0];
    tmpN_Point.normal_y = coefficients->values[1];
    tmpN_Point.normal_z = coefficients->values[2];
    tmpN_Point.curvature = 1;

    centroidNormalCloud->push_back(tmpN_Point);

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*fCloud);
    remainingCloud.swap(fCloud);
    i++;
  }

  return  centroidNormalCloud;
}

pcl::PointCloud<pcl::Normal>::Ptr getCloudNormals(PointCloud::Ptr input, float radius){

  // ESTIMATE NORMALS //////////////////////////////////////////////////////////
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

  pcl::PointCloud<pcl::Normal>::Ptr cloudNoramls (new pcl::PointCloud<pcl::Normal>);  

  ne.setInputCloud(input);
  ne.setSearchMethod (tree);
  ne.setRadiusSearch(radius);
  ne.compute(*cloudNoramls);

  std::cout << "Input Size:" << input->size() << '\n' << "Cloud Normals Size: " <<
  cloudNoramls->size() << std::endl;  

  //pcl::visualization::PCLVisualizer normalVis ("Normals Visualizer");
  //normalVis.addPointCloudNormals<PointT, pcl::Normal> (input, cloudNormals);

  return cloudNoramls;
}

// REGISTRATION //

Eigen::Matrix4f icpDefault(PointCloud::Ptr source, PointCloud::Ptr target){

  pcl::IterativeClosestPoint<PointT, PointT> icp;

  PointCloud tmpOutputCloud;
  //icp.setTransformationEstimation();
  //icp.setMaxCorrespondenceDistance(0.01);
  icp.setInputSource(source);
  icp.setInputTarget(target);
  icp.setMaximumIterations(100000);
  //icp.setTransformationEpsilon(1e-20);
  icp.setEuclideanFitnessEpsilon(0.0000001);

  icp.align(tmpOutputCloud);

  Eigen::Matrix4f mth;
  mth = icp.getFinalTransformation();

  std::cout << "Has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  //std::cout << icp.getFinalTransformation() << std::endl;

  return mth;
}

Eigen::Matrix4f icpWithNormals(PointCloudWithNormals::Ptr source, PointCloudWithNormals::Ptr target){

  pcl::IterativeClosestPoint<PointNormalT, PointNormalT> icp;

  PointCloudWithNormals tmpOutputCloud;
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

Eigen::Matrix4f icpWithIntensity(PointCloudIntensity::Ptr source, PointCloudIntensity::Ptr target){

  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;

  PointCloudIntensity tmpOutputCloud;
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

Eigen::Matrix4f icpGeneralized(PointCloud::Ptr source, PointCloud::Ptr target){

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

