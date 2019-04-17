#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <iostream>

using namespace std; 

ros::Publisher pub;

void callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
	pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl (new pcl::PointCloud<pcl::PointXYZ>);

    //Using the PointCloud2 data from the ROS topic
    pcl_conversions::toPCL(*input, *cloud);
    pcl::fromPCLPointCloud2 (*cloud, *cloud_pcl);

    pcl::io::savePCDFileASCII("newstscan.pcd", *cloud_pcl);
    cout << "Saved" << endl;
}

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "reading_pcd");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/kinect2/sd/points", 1, callback);

    // Create a ROS publisher for the output model coefficients
    //pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

    // Spin
    ros::spin ();
}
