
// Cpp
#include <iostream>

// PCL
#include <pcl/PCLPointCloud2.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/PointIndices.h>

// PCL Filters
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>


typedef pcl::PointCloud<pcl::PointXYZ> cloudXYZ;
typedef pcl::PCLPointCloud2 pc2;
pcl::visualization::PCLVisualizer pcl_viewer ("PCL_Visualizer");
pcl::visualization::CloudViewer cloud_viewer ("Cloud_Viewer");


int main (){
  cloudXYZ::Ptr outpuCloud (new cloudXYZ);

  // Handlers for InputData and OutputData
  pc2::Ptr cloud_blob (new pc2), cloud_filtered_blob (new pc2);
  cloudXYZ::Ptr cloud_filtered (new cloudXYZ), cloud_p (new cloudXYZ), cloud_f (new cloudXYZ);

  // Fill in the cloud data
  pcl::PCDReader reader;
  reader.read ("table_scene_lms400.pcd", *cloud_blob);

  std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height 
            << " data points." << std::endl;

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pc2> sor;
  sor.setInputCloud (cloud_blob);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered_blob);

  // Convert to the templated PointCloud
  pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
            << " data points." << std::endl;

  // Write the downsampled version to disk
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);


  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
    return (-1);
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

  pcl::ExtractIndices<pcl::PointXYZ> extract;  
  
  extract.setInputCloud(cloud_filtered);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*outpuCloud);

  cloud_viewer.showCloud(outpuCloud);

  while (!cloud_viewer.wasStopped()){
    int a;
    a = 1;
  }


  return (0);
}