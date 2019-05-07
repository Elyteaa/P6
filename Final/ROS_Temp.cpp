//PCL
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
//c++

#include <math.h>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

//PCL ROS
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>

//ROS
#include "std_msgs/Float32MultiArray.h"
//Might not need these
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

using namespace std;

ros::Publisher pub;
//Functions here//
void callBack(const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::PCLPointCloud2::Ptr cloud_in(new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    //Using the PointCloud2 data from the ROS topic
    pcl_conversions::toPCL(*input, *cloud_in);

    // Convert to the templated PointCloud
    pcl::fromPCLPointCloud2(*cloud_in, *cloud);
    /////////////
    
    //Code here//
    
    /////////////
    
    std_msgs::Float32MultiArray output;
   output.data.clear();

    for(int i = 0; i < 6; i++)
    {
        output.data.push_back(pose1[i]);
    }


    pub.publish (output);
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "my_pcl_tutorial");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/kinect2/sd/points", 1, callBack);

    // Create a ROS publisher for the output model coefficients
    pub = nh.advertise<std_msgs::Float32MultiArray>("/robotPose", 1000);




    // Spin
    ros::spin();

    return 0;
}
