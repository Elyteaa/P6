//ROS
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>
//PCL
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/moment_of_inertia_estimation.h>

using namespace std;

void callBack(const sensor_msgs::PointCloud2ConstPtr& input)
{

//Setting up our clouds
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ (new pcl::PointCloud<pcl::PointXYZ>);

//Converting the clouds to the correct formats
  pcl_conversions::toPCL(*input, *cloud);
  pcl::fromPCLPointCloud2 (*cloud, *cloudXYZ);
  cout << "Cloud has been converted" << endl;

//What we need in order to set up the bounding boxes
  pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud(cloudXYZ);
  feature_extractor.compute();
  pcl::PointXYZ min_point_AABB;
  pcl::PointXYZ max_point_AABB;
  pcl::PointXYZ min_point_OBB;
  pcl::PointXYZ max_point_OBB;
  pcl::PointXYZ position_OBB;
  Eigen::Matrix3f rotational_matrix_OBB;
  feature_extractor.getAABB (min_point_AABB, max_point_AABB);
  feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

  cout << "Features extracted" << endl;

//Setting up the viewer
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer("viewer", false));
  viewer->setBackgroundColor (1, 1, 1);
  viewer->addCoordinateSystem (1.0f, "global");
  cout << "Setting up viewer" << endl;

//Adding the pointcloud to the viewer

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> point_cloud_color_handler (cloudXYZ, 0, 0, 0);
  viewer->addPointCloud(cloudXYZ, point_cloud_color_handler, "original point cloud");
 // viewer.addPointCloud<pcl::PointXYZ>(cloudXYZ, "sample cloud");
 cout << "Adding pointcloud to viewer" << endl;

//Adding our first bounding box defined with AABB
  viewer->addCube (min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 242, 242, 33, "AABB");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");

cout << "First bounding box added" << endl;

//Setting up the needed stuff for the OBB bounding box
  Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
  Eigen::Quaternionf quat (rotational_matrix_OBB);

//Adding the OBB bounding box
  viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");

cout << "Second bounding box added" << endl;

//Running the viewer in a loop
while(!viewer->wasStopped ())
{ 
  viewer->spinOnce (100);

}
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/kinect2/sd/points", 1, callBack);
  ros::spin ();
}
