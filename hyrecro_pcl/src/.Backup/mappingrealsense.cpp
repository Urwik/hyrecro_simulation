/*
  REGISTRA POR PARES 2 NUBES DE PUNTOS ESPECIFICADAS DENTRO DEL MAIN()
  CREA UN MAPA INTEGRANDO NUBES DE PUNTOS 2 A 2 DESDE ARCHIVOS  .pcd GUARDADOS.
*/
// cpp
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry> 
#include <chrono>
#include <cmath>
#include <dirent.h>

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
#include <pcl/visualization/cloud_viewer.h>


//****************************************************************************//
// TYPE DEFINITIONS ////////////////////////////////////////////////////////////

typedef pcl::PointXYZRGB PointT;
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

PointCloudWithNormals::Ptr  getCloudNormals(PointCloud::Ptr input, float radius);


// REGISTRATION //

Eigen::Matrix4f icpDefault(PointCloud::Ptr src, PointCloud::Ptr tgt);

Eigen::Matrix4f icpWithNormals(PointCloud::Ptr src, PointCloud::Ptr tgt);

Eigen::Matrix4f icpGeneralized(PointCloud::Ptr src, PointCloud::Ptr tgt);

Eigen::Matrix4f icpNonLinear(PointCloud::Ptr src, PointCloud::Ptr tgt);

Eigen::Matrix4f icpWithIntensity(PointCloudIntensity::Ptr source, PointCloudIntensity::Ptr target);

Eigen::Matrix4f icp2Phase(PointCloud::Ptr source, PointCloud::Ptr target);


////////////////////////////////////////////////////////////////////////////////

std::vector<double> fitScore;


int main(int argc, char **argv)
{

  // Get handlres for source and target cloud data /////////////////////////////
  PointCloud::Ptr source (new PointCloud);
  PointCloud::Ptr target (new PointCloud);

  PointCloud::Ptr alignedSource (new PointCloud);
  PointCloud::Ptr mapCloud (new PointCloud);

  // pcl::visualization::PCLVisualizer::Ptr visOriginal (new pcl::visualization::PCLVisualizer ("Original Visualizer"));
  // pcl::visualization::PCLVisualizer::Ptr visTF (new pcl::visualization::PCLVisualizer ("Transformed Visualizer"));
  pcl::visualization::PCLVisualizer::Ptr visMap (new pcl::visualization::PCLVisualizer ("Map Visualizer"));

  CloudColor srcColor   (source,   255,    0,    0);
  CloudColor tgtColor   (target,     0,  255,    0);
  CloudColor algColor   (alignedSource, 255,    0,    0);
  CloudColor mapColor   (mapCloud, 0,    200,    255);


  std::stringstream ss;
  pcl::PCDReader reader;

  Eigen::Matrix4f tmpMth;
  Eigen::Matrix4f globalMth = Eigen::Matrix4f::Identity();

  auto start = std::chrono::high_resolution_clock::now();

  for (int i = 0; i <=1128; i=i+1)
  {
    ss.str("");
    ss << i << ".pcd";
    reader.read(ss.str(), *target);
    
    ss.str("");
    ss << i+1 << ".pcd";
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
    //tmpMth = icpDefault(source, target);
    //tmpMth = icpWithNormals(source, target);
    tmpMth = icpGeneralized(source, target);
    //tmpMth = icp2Phase(source, target);
    //tmpMth = icpNonLinear(source, target);

    // TRANSFORMATION
    globalMth = globalMth * tmpMth;
    
    pcl::transformPointCloud (*source, *alignedSource, globalMth);

    *mapCloud += *alignedSource;
    sor.setInputCloud(mapCloud);
    sor.filter(*mapCloud);


    visMap->removeAllPointClouds();
    visMap->setBackgroundColor(50,50,50);
    visMap->addPointCloud<PointT> (mapCloud, mapColor, "MapCloud");
    visMap->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "MapCloud");

    visMap->spinOnce();    
  }
  
  std::cout << "REGISTRATION ENDED" << std::endl;

  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop-start);
  std::cout << "Total time: " << duration.count() << std::endl;

  double sum = std::accumulate(fitScore.begin(), fitScore.end(), 0.0);
  double mean = sum / fitScore.size();
  std::cout << "Med Fit Score: " << mean << std::endl;
  double max = *std::max_element(fitScore.begin(), fitScore.end());
  std::cout << "Max error: " << max << std::endl;

  pcl::PCDWriter writer;
  writer.write<PointT>("map.pcd", *mapCloud, false);


  while(!visMap->wasStopped())
    visMap->spinOnce(100);

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
  //icp.setTransformationEstimation();

  //icp.setMaxCorrespondenceDistance(0.1);
  //icp.setRANSACOutlierRejectionThreshold(0.0002);

  // icp.setMaximumIterations(10000);
  // icp.setTransformationEpsilon(1e-20);  //Convergence criteria
  // icp.setEuclideanFitnessEpsilon(1e-5); //Divergence criteria

  icp.align(*result);

  Eigen::Matrix4f mth;
  mth = icp.getFinalTransformation();

  fitScore.push_back(icp.getFitnessScore());
  //std::cout <<  "Score: "     << icp.getFitnessScore()  << std::endl;

  return mth;
}

Eigen::Matrix4f icpWithNormals(PointCloud::Ptr src, PointCloud::Ptr tgt){
  PointCloudWithNormals::Ptr srcWithNormals (new PointCloudWithNormals);
  PointCloudWithNormals::Ptr tgtWithNormals (new PointCloudWithNormals);

  pcl::IterativeClosestPointWithNormals<PointNormalT, PointNormalT> icpWN;

  PointCloudWithNormals tmpOutputCloud;

  srcWithNormals = getCloudNormals(src, 0.05);
  tgtWithNormals = getCloudNormals(tgt, 0.05);


  //icp.setTransformationEstimation();
  //icp.setMaxCorrespondenceDistance(0.01);
  icpWN.setInputSource(srcWithNormals);
  icpWN.setInputTarget(tgtWithNormals);
  //icpWithNormals.setMaximumIterations(1000);
  //icp.setTransformationEpsilon(1e-20);
  //icp.setEuclideanFitnessEpsilon(0.001);

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

Eigen::Matrix4f icpNonLinear(PointCloud::Ptr src, PointCloud::Ptr tgt){
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> icpNl;
  
  PointCloudWithNormals::Ptr srcWithNormals (new PointCloudWithNormals);
  PointCloudWithNormals::Ptr tgtWithNormals (new PointCloudWithNormals);
  PointCloudWithNormals::Ptr result;
  PointNormalT punto;

  srcWithNormals = getCloudNormals(src, 0.05);
  tgtWithNormals = getCloudNormals(tgt, 0.05);

  //icpNl.setPointRepresentation(pcl::make_shared<const PointNormalT> (punto));
  icpNl.setTransformationEpsilon (1e-6);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  icpNl.setMaxCorrespondenceDistance (0.1);  
  // Set the point representation

  icpNl.setInputSource (srcWithNormals);
  icpNl.setInputTarget (tgtWithNormals);

  icpNl.setMaximumIterations (30);

  icpNl.align (*result);

  Eigen::Matrix4f mth;
  mth = icpNl.getFinalTransformation();
  fitScore.push_back(icpNl.getFitnessScore());
  
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

Eigen::Matrix4f icp2Phase(PointCloud::Ptr source, PointCloud::Ptr target){
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  PointCloud tmpOutputCloud;


  // ICP WITH THE ESTIMATION //
  Eigen::Matrix<float, 4, 4> estMth;
  pcl::registration::TransformationEstimation<PointT, PointT>::Ptr estimation;
  estimation->estimateRigidTransformation(*source, *target, estMth);

  icp.setTransformationEstimation(estimation);
  icp.setInputSource(source);
  icp.setInputTarget(target);
  //icp.setTransformationEstimation();

  //icp.setMaxCorrespondenceDistance(0.1);
  //icp.setRANSACOutlierRejectionThreshold(0.0002);

  //icp.setMaximumIterations(10000);
  //icp.setTransformationEpsilon(1e-20);  //Convergence criteria
  //icp.setEuclideanFitnessEpsilon(1e-5); //Divergence criteria

  icp.align(tmpOutputCloud);

  Eigen::Matrix4f mth;
  mth = icp.getFinalTransformation();

  fitScore.push_back(icp.getFitnessScore());

  return mth;
}

