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
#include <pcl/visualization/pcl_visualizer.h>
 
ros::Publisher pub;

void callBack (const sensor_msgs::PointCloud2ConstPtr& input)
{
	pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_to_use (new pcl::PointCloud<pcl::PointXYZ>);

    pcl_conversions::toPCL(*input, *cloud_blob);
    pcl::fromPCLPointCloud2 (*cloud_blob, *cloud_to_use);

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("SimpleCloudViewer"));
    viewer->setBackgroundColor(0,0,0);
	viewer->addPointCloud<pcl::PointXYZ> (*cloud_to_use, "sampleCloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sampleCloud");
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();

	while (!viewer->wasStopped ())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "my_pcl_tutorial");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/kinect2/sd/points", 1, callBack);

    // Create a ROS publisher for the output model coefficients
    pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

    // Spin
    ros::spin ();
}