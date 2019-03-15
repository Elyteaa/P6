#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/transforms.h>
#include <iostream>

ros::Publisher pub;

void callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
	pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl (new pcl::PointCloud<pcl::PointXYZ>);

    //Using the PointCloud2 data from the ROS topic
    pcl_conversions::toPCL(*input, *cloud);
    pcl::fromPCLPointCloud2 (*cloud, *cloud_pcl);

    pcl::io::savePCDFileASCII("nice_leg1.pcd", *cloud_pcl);
    cout << "Saved" << endl;
}