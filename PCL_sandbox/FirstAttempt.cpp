#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  //pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>),   cloud_p  (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;
  //pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl_conversions::toPCL(*input, *cloud);

  //Filtering is performed here, to reduce the number of data points
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter(cloud_filtered);

  //pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);
	
  //Finding the coefficients
  pcl::ModelCoefficients coefficients;
  pcl::PointIndices::Ptr inliers (new pcl::ModelCoefficients());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);

  pcl::ExtractIndices<pcl::PointXYZ> extract;

int i = 0, nr_points = (int)cloud_filtered->points.size();
while (cloud_filtered->points.size() > 0.3 * nr_points)
{	

  seg.setInputCloud(cloud_filtered);
  seg.segment(*inliers, *coefficients);

  extract.setInputCloud(cloud_filtered);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*cloud_p);

  extract.setNegative(true);
  extract.filter(*cloud);
  cloud_filtered.swap(cloud);
  i++;
}
  
  // Publish the model coefficients
  //pcl_msgs::PointCloud2 ros_cloud;
  sensor_msgs::PointCloud2 ros_cloud;
  pcl_conversions::fromPCL(cloud, ros_cloud);
  pub.publish(ros_cloud);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output model coefficients
  pub = nh.advertise<pcl_msgs::ModelCoefficients> ("output", 1);

  // Spin
  ros::spin ();
}
