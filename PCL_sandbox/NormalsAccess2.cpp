#include <iostream>
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
  cout << "h" << endl;
  // Setup the principal curvatures computation
  pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> principalCurvaturesEstimation;
  cout << "e" << endl;
  // Provide the original point cloud (without normals)
  principalCurvaturesEstimation.setInputCloud(final_cloud);
  cout << "r" << endl;
  // Provide the point cloud with normals
  principalCurvaturesEstimation.setInputNormals(normals);
  cout << "e" << endl;
  // Use the same KdTree from the normal estimation
  principalCurvaturesEstimation.setSearchMethod (tree);
  principalCurvaturesEstimation.setRadiusSearch(1.0);
  cout << "?" << endl;
  // Actually compute the principal curvatures
  pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principalCurvatures (new pcl::PointCloud<pcl::PrincipalCurvatures> ());
  principalCurvaturesEstimation.compute(*principalCurvatures);
  cout << "!" << endl;
  cout << "output points.size (): " << principalCurvatures->points.size () << endl;

  // Display and retrieve the shape context descriptor vector for the 0th point.
  pcl::PrincipalCurvatures descriptor = principalCurvatures->points[0];
  cout << descriptor << endl;
 // cout <<  << endl;
/*
  pcl::CentroidPoint<pcl::PointXYZ> cen;
  cen.add(*final);
  cen.add(*normals);
  */
/*
  pcl::PointCloud<pcl::NormalT>::Ptr normals_ref (new pcl::PointCloud<pcl::NormalT>);
  pcl::NormalRefinement<NormalT> nr;
  nr.setInputCloud(normals);
  nr.filter(normals_ref);
*/
  //pass.setFilterLimitsNegative (true);
  
  //pcl::io::savePCDFileASCII("goodie.pcd", *final);
 // pcl::io::savePLYFileBinary("trasncleaner.ply", *final);
  //  cout << "Saved" << endl;

//normals
 // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  //ne.setInputCloud (cloud_filtered);
/*
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
  //Attempting to do some ransac

  // Create a shared plane model pointer directly
  //pcl::PointCloud<pcl::Normal>Ptr normie (new pcl::PointCloud<pcl:Normal>);
  pcl::SampleConsensusModelCylinder<pcl::PointXYZ, pcl::Normal>::Ptr model (new pcl::SampleConsensusModelCylinder<pcl::PointXYZ, pcl::Normal>(cloud_filtered));
  //SampleConsensusModelPlane<PointXYZ>::Ptr model (new SampleConsensusModelPlane<PointXYZ> (cloud_filtered));

  // Create the RANSAC object
  pcl::RandomSampleConsensus<pcl::PointXYZ> sac(model, 0.03);
  //RandomSampleConsensus<PointXYZ> sac (model, 0.03);

  // perform the segmenation step
  bool result = sac.computeModel ();

  // get inlier indices
boost::shared_ptr<vector<int> > inliers (new vector<int>);
sac.getInliers (*inliers);
cout << "Found model with " << inliers->size () << " inliers";


// get model coefficients
Eigen::VectorXf coeff;
sac.getModelCoefficients (coeff);
cout << ", plane normal is: " << coeff[0] << ", " << coeff[1] << ", " << coeff[2] << endl;

// perform a refitting step
Eigen::VectorXf coeff_refined;
model->optimizeModelCoefficients
(*inliers, coeff, coeff_refined);
model->selectWithinDistance(coeff_refined, 0.03, *inliers);
cout << "After refitting, model contains " << inliers->size () << " inliers"; 
cout << ", plane normal is: " << coeff_refined[0] << ", " << coeff_refined[1] << ", " << coeff_refined[2] << "." << endl;

// Projection
//PointCloud<PointXYZ> proj_points;
pcl::PointCloud<pcl::PointXYZ>::Ptr proj_points (new pcl::PointCloud<pcl::PointXYZ>);

model->projectPoints(*inliers, coeff_refined, *proj_points);



/*
std::vector<int> inliers;

  // created RandomSampleConsensus object and compute the appropriated model
  pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr
    model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(cloud_filtered));
  pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
    model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud_filtered));
 
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
    ransac.setDistanceThreshold (.01);
    ransac.computeModel();
    ransac.getInliers(inliers);
  

  // copies all inliers of the model computed to another PointCloud
  pcl::copyPointCloud<pcl::PointXYZ>(*cloud_filtered, inliers, *final);
*/



pcl::visualization::PCLVisualizer::Ptr viewer;
viewer = simpleVis(final_cloud, normals);
while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }




/*


  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (final, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }*/



  // creates the visualization object and adds either our original cloud or all of the inliers
  // depending on the command line arguments specified.
  /*pcl::visualization::PCLVisualizer::Ptr viewer;
  if (pcl::console::find_argument (argc, argv, "-f") >= 0 || pcl::console::find_argument (argc, argv, "-sf") >= 0)
    viewer.addPointCloud<pcl::PointXYZ> (final, "name2");
  else
    viewer.addPointCloud<pcl::PointXYZ> (cloud, "name");*/
/*
  pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud (cloud_filtered);
  feature_extractor.compute ();

  std::vector <float> moment_of_inertia;
  std::vector <float> eccentricity;
  pcl::PointXYZ min_point_AABB;
  pcl::PointXYZ max_point_AABB;
  pcl::PointXYZ min_point_OBB;
  pcl::PointXYZ max_point_OBB;
  pcl::PointXYZ position_OBB;
  Eigen::Matrix3f rotational_matrix_OBB;
  float major_value, middle_value, minor_value;
  Eigen::Vector3f major_vector, middle_vector, minor_vector;
  Eigen::Vector3f mass_center;

  feature_extractor.getMomentOfInertia (moment_of_inertia);
  feature_extractor.getEccentricity (eccentricity);
  feature_extractor.getAABB (min_point_AABB, max_point_AABB);
  feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
  feature_extractor.getEigenValues (major_value, middle_value, minor_value);
  feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
  feature_extractor.getMassCenter (mass_center);

  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  viewer->addPointCloud<pcl::PointXYZ> (cloud_filtered, "sample cloud");
  viewer->addCube (min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");

  Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
  Eigen::Quaternionf quat (rotational_matrix_OBB);
  viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");

  pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
  pcl::PointXYZ x_axis (major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
  pcl::PointXYZ y_axis (middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
  pcl::PointXYZ z_axis (minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));
  viewer->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
  viewer->addLine (center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
  viewer->addLine (center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");

  while(!viewer->wasStopped())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
*/
//pcl::io::savePCDFileASCII ("nicelegfilter.pcd", *cloud);
//std::cout << "saved" << std::endl;
  return (0);
}

