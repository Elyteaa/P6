     #include <pcl/io/io.h>
     #include <pcl/io/pcd_io.h>
     #include <pcl/features/integral_image_normal.h>
     #include <pcl/visualization/cloud_viewer.h>
	#include <pcl/filters/voxel_grid.h>
	#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d.h>
#include <iostream>
#include <pcl/filters/passthrough.h>

using namespace std;
     int main()
     {
             // load point cloud
             pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
             
             pcl::io::loadPCDFile ("nice_leg3.pcd", *cloud);
			 //pcl::PCLPointCloud2::Ptr cloud2 (new pcl::PCLPointCloud2 ());
			 /*cloud->width = 185;
             cloud->height = 120;
             cloud->points.resize(cloud->width * cloud->height);
             cloud->is_dense = false;
			*/
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
   pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile <pcl::PointXYZ> ("nice_leg3.pcd", *cloud);


  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits(0.6, 0.9);

  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);

//VoxelGrid
   //pcl::toPCLPointCloud2(*cloud, *cloud2);
   //pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
   //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
   //cloud_final->width = 640;
   //cloud_final->height = 480;
   //pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
   //sor.setInputCloud(cloud2);
   //sor.setLeafSize(10, 10, 10);
   //sor.filter(*cloud_filtered);
   //std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
   //    << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";
   //pcl::fromPCLPointCloud2(*cloud_filtered, *cloud_final);


             // estimate normals
/*             
             pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

             pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
             ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
             ne.setMaxDepthChangeFactor(0.02f);
             ne.setNormalSmoothingSize(10.0f);
             ne.setInputCloud(cloud_filtered);
             ne.compute(*normals);

  */           
             // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud_filtered);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.03);

  // Compute the features
  ne.compute (*normals);
  
             // visualize normals
             pcl::visualization::PCLVisualizer viewer("PCL Viewer");
             viewer.setBackgroundColor (0.0, 0.0, 0);
             viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud_filtered, normals);
             viewer.addPointCloud<pcl::PointXYZ> (cloud_filtered, "sample cloud");

             while(!viewer.wasStopped ())
             {


                 //ne.setInputCloud(cloud);
               viewer.spinOnce ();
             }
              //pcl::io::savePCDFileASCII ("leg.pcd", *cloud);
             return 0;
     }
