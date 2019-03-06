/*Way to read .ply file (original pig pointcloud) have the .ply
file in the same directory as your main.cpp, compile with cmake .. and make
and then run the executable  */


#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>


int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2::Ptr cloud2 (new pcl::PCLPointCloud2 ());
    //pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  
 

  if (pcl::io::loadPLYFile<pcl::PointXYZ> ("pigleg.ply", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }


  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;

   pcl::toPCLPointCloud2(*cloud, *cloud2);
   pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloud2);
  sor.setLeafSize(10, 10, 10);
  sor.filter(*cloud_filtered);
  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";
  pcl::fromPCLPointCloud2(*cloud_filtered, *cloud_final);

  pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
  viewer.showCloud(cloud_final);
   while(!viewer.wasStopped ())
   {
  /*for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cout << "    " << cloud->points[i].x
            << " "    << cloud->points[i].y
              << " "    << cloud->points[i].z << std::endl;
}*/

}
  return (0);
}
