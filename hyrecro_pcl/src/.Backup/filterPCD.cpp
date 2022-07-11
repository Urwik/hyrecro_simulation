/*

This code is used to resample and filter .pcd files
argv[1] = extension to save the PointCloud "pcd" or "ply"
argv[2] = name of the file to filter
*/
// C++
#include <iostream>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

  // PCL FILTERS
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

int main(int argc, char **argv)
{
  // Read the pcd file
  pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud (new pcl::PointCloud<pcl::PointXYZ>);
  std::string outType = argv[1];
  std::string fileName = argv[2];
    
  pcl::PCDReader reader;
  int tmp = reader.read(fileName, *inCloud);

  if (tmp==0)
    std::cout << "File read correctly" << '\n';
  else
    std::cout << "Error reading the file" << '\n';


  // Filter Height
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(inCloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-1, 1);
  pass.filter(*inCloud);

  
  pcl::PointCloud<pcl::PointXYZ>::Ptr tmpY (new pcl::PointCloud<pcl::PointXYZ>);

  pass.setInputCloud(inCloud);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-2, 2);
  pass.filter(*inCloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr tmpX (new pcl::PointCloud<pcl::PointXYZ>);

  pass.setInputCloud(inCloud);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(-2, 2);
  pass.filter(*inCloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud (new pcl::PointCloud<pcl::PointXYZ>);

  *outCloud = *inCloud; 

  // Voxel Filter
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(outCloud);
  sor.setLeafSize(0.1, 0.1, 0.1);
  sor.filter(*outCloud);


  // Set name of the file
  size_t lastindex = fileName.find_last_of("."); 
  std::string rawname = fileName.substr(0, lastindex); 


  std::stringstream ss;
  ss.str("");

  if (outType == "ply"){
    ss << rawname << "_sampled.ply";
    pcl::PLYWriter plyWriter;
    plyWriter.write(ss.str(), *outCloud);
  }
  else if (outType == "pcd"){
    ss << rawname << "_sampled.pcd";
    pcl::PCDWriter writer;
    tmp = writer.write(ss.str(), *outCloud);
  }

  std::cout << "File saved with name: "<< ss.str() << '\n';

}