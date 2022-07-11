/*
  REGISTRA POR PARES 2 NUBES DE PUNTOS ESPECIFICADAS DENTRO DEL MAIN()
*/
// cpp
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>


  // PCL VISUALIZATION
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>

#include <chrono>
#include <thread>


//****************************************************************************//
// TYPE DEFINITIONS ////////////////////////////////////////////////////////////

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

typedef pcl::PointCloud<pcl::PointXYZI> PointCloudIntensity;

using namespace std::this_thread; // sleep_for, sleep_until
using namespace std::chrono; // nanoseconds, system_clock, seconds


// ************************************************************************** //


// ************************************************************************** //
// AVAILABLE FUNCTIONS /////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////


int main(int argc, char **argv)
{

  // Get handlres for source and target cloud data /////////////////////////////

  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Original Visualizer"));
  pcl::PCDReader reader;

  PointCloud::Ptr cloud (new PointCloud);

  std::stringstream ss;

  for (int i=1; i <=73; i++){
    ss.str("");
    ss << i << ".pcd";
    reader.read(ss.str(), *cloud);
    viewer->addPointCloud<PointT>(cloud, ss.str());
    viewer->spinOnce (100);
    sleep_for(nanoseconds(1000));

  }
  while(!viewer->wasStopped())
  {
    viewer->spinOnce (100);
  }
  return 0;
}
  
  
