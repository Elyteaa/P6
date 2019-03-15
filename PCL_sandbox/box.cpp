/*Way to read .ply file (original pig pointcloud) have the .ply
file in the same directory as your main.cpp, compile with cmake .. and make
and then run the executable  */


#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/moment_of_inertia_estimation.h>


int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2::Ptr cloud2 (new pcl::PCLPointCloud2 ());
    //pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  
 

  if (pcl::io::loadPLYFile<pcl::PointXYZ> ("legcloud.ply", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }


  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
/*
   pcl::toPCLPointCloud2(*cloud, *cloud2);
   pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloud2);
  sor.setLeafSize(10, 10, 10);
  sor.filter(*cloud_filtered);
  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";
  pcl::fromPCLPointCloud2(*cloud_filtered, *cloud_final);*/
//Test stuff
   pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud (cloud);
  feature_extractor.compute ();
  pcl::PointXYZ min_point_AABB;
  pcl::PointXYZ max_point_AABB;
  pcl::PointXYZ min_point_OBB;
  pcl::PointXYZ max_point_OBB;
  pcl::PointXYZ position_OBB;
Eigen::Matrix3f rotational_matrix_OBB;
  feature_extractor.getAABB (min_point_AABB, max_point_AABB);
  feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
//End of test stuff
 // pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
 // viewer.showCloud(cloud_final);
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
 viewer->addCoordinateSystem (1.0);
 viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->addCube (min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 242, 242, 33, "AABB");
viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");
 Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
  Eigen::Quaternionf quat (rotational_matrix_OBB);
  viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");
   while(!viewer->wasStopped ())
   { 
   viewer->spinOnce (100);
  /*for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cout << "    " << cloud->points[i].x
            << " "    << cloud->points[i].y
              << " "    << cloud->points[i].z << std::endl;
}*/

}
  return (0);
}
