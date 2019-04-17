#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <vector>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/filters/normal_refinement.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>

using namespace std;

pcl::visualization::PCLVisualizer::Ptr
simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem(0.1, 0, 0, 0);
  //viewer->addCoordinateSystem (1.0, "global");
 // viewer->initCameraParameters ();
  return (viewer);
}

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2);
  pcl::PCLPointCloud2::Ptr cloud_after (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile <pcl::PointXYZ> ("goodscan.pcd", *cloud);


  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits(0.30, 1.03);

  //pass.setFilterLimitsNegative (true);
  pass.filter(*cloud_filtered);

//normals
  

  // pcl::PassThrough<pcl::PointXYZ> pass1;
  pass.setInputCloud(cloud_filtered);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits(-0.30, 0.20);

  pass.filter(*final_cloud);

  pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud(final_cloud);
  feature_extractor.compute();

  std::vector <float> moment_of_inertia;
  std::vector <float> eccentricity;
  float major_value, middle_value, minor_value;
  Eigen::Vector3f major_vector, middle_vector, minor_vector;
  Eigen::Vector3f mass_center;
  feature_extractor.getMomentOfInertia (moment_of_inertia);
  feature_extractor.getEccentricity (eccentricity);
  feature_extractor.getEigenValues (major_value, middle_value, minor_value);
  feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
  feature_extractor.getMassCenter (mass_center);

  pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
  pcl::PointXYZ x_axis (major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
  pcl::PointXYZ y_axis (middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
  pcl::PointXYZ z_axis (minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));

  cout << "Mass center = " << center << "\nMajor vector = " << x_axis << "\nMiddle_vector = " << y_axis << "\nMinor_vector = " << z_axis << endl;

  //Voxel grid filter//
  //pcl::toPCLPointCloud2(*final, *cloud_blob);
  /*
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloud_blob);
  sor.setLeafSize (0.001, 0.001, 0.001);
  sor.filter(*cloud_after);
*/
  //pcl::fromPCLPointCloud2(*cloud_after, *final_cloud);


    


  
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(final_cloud);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  


  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.03);

  // Compute the features
  ne.compute(*normals);
  cout << "output points.size (): " << normals->points.size() << endl;

  

  float manynormals;
  float max;

  for(int i = 0; i < normals->points.size(); i++)
  {
  // Display and retrieve the shape context descriptor vector for the 0th point.
  //pcl::PrincipalCurvatures descriptor = principalCurvatures->points[i];

  if(normals->points[i].normal_z < 0 || normals->points[i].normal_z > 0){
    normals->points[i].normal_z = 0;
  }
  //max = normals->points[0].curvature;
  if(max < normals->points[i].curvature)
  {
    max = normals->points[i].curvature;
  }
  //cout << manynormals << endl;
  //cout << max << endl;
  }

  /*
  // Setup the principal curvatures computation
  pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> principalCurvaturesEstimation;
  // Provide the original point cloud (without normals)
  principalCurvaturesEstimation.setInputCloud(final_cloud);
  // Provide the point cloud with normals
  principalCurvaturesEstimation.setInputNormals(normals);
  // Use the same KdTree from the normal estimation
  principalCurvaturesEstimation.setSearchMethod (tree);
  principalCurvaturesEstimation.setRadiusSearch(1.0);
  // Actually compute the principal curvatures
  pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principalCurvatures (new pcl::PointCloud<pcl::PrincipalCurvatures> ());
  principalCurvaturesEstimation.compute(*principalCurvatures);
  //cout << "output points.size (): " << principalCurvatures->points.size () << endl;
  */

  

 for(int j = 0; j < final_cloud->points.size(); j++)
  {
    if(final_cloud->points[j].z < 0 || final_cloud->points[j].z > 0){
    final_cloud->points[j].z = 1;
  
    //cout << "x = " << final_cloud->points[j].x << " y = " << final_cloud->points[j].y << " z = " << final_cloud->points[j].z << endl;
}
}
pcl::PCDWriter writer;
  writer.write ("flat.pcd", *final_cloud, false);

pcl::visualization::PCLVisualizer::Ptr viewer;
viewer = simpleVis(final_cloud, normals);
while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

  return (0);
}

