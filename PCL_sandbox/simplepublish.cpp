//ROS
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>
#include "std_msgs/Int32MultiArray.h"
//Might not need these
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
//PCL
#include <pcl/PCLPointCloud2.h>
#include <vector>
#include <tuple>
#include <pcl/io/ply_io.h>
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

ros::Publisher pub;

//tuple<int, int> eighteen(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int start, int end);

tuple<int, int> eighteen(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int start, int end)
{

double distx = cloud->points[start].x - cloud->points[end].x;
double disty = cloud->points[start].y - cloud->points[end].y;
double distz = cloud->points[start].z - cloud->points[end].z;
//Finding the length of the line
double len1 = sqrt(pow(distx, 2.0) + pow(disty, 2.0) + pow(distz, 2.0));

double difflen = 0.14/len1;

//Scaling the line to 0.18
len1 = len1 * difflen;

//By modifying t we can find any point on the line
//where t = 0 corresponds to our starting point and t = 1 to our end point
//any value of t in between will be a point on the line between start and end
double t = -1 * difflen;
//Here we calculate the direction "vector"
double directionx = cloud->points[start].x - cloud->points[end].x;
double directiony = cloud->points[start].y - cloud->points[end].y;
double directionz = cloud->points[start].z - cloud->points[end].z;


//Pushing the point on the line 18cm from start to the pointcloud
//Notice that each point is calculated by adding the start point with the direction vector multiplied by t
pcl::PointXYZ pp;
pp.x = cloud->points[start].x + directionx * t; pp.y = cloud->points[start].y + directiony * t; pp.z = cloud->points[start].z + directionz * t;
cloud->push_back(pp);

cout << cloud->points.size() << endl;
//Here we find the distance from the start of the line to the point
int lastindex = cloud->points.size()-1;
double distpointx = cloud->points[start].x - cloud->points[lastindex].x;
double distpointy = cloud->points[start].y - cloud->points[lastindex].y;
double distpointz = cloud->points[start].z - cloud->points[lastindex].z;
double lenToPoint = sqrt(pow(distpointx, 2.0) + pow(distpointy, 2.0) + pow(distpointz, 2.0));
double lenToPoint1 = lenToPoint;
//Here we search for the smallest distance to the 18cm point from any point in the pointclooud
//The shortst distance to the point will then be the last i value -1 since the smallest distance to the point is the distance from the point to the itself
int p1 = 0;
int p2 = 0;
vector<int> temp;
for(int i = 0; i < cloud->points.size(); i++)
{
  double itepointx = cloud->points[i].x - cloud->points[lastindex].x;
  double itepointy = cloud->points[i].y - cloud->points[lastindex].y;
  double itepointz = cloud->points[i].z - cloud->points[lastindex].z;

  if(lenToPoint != 0 && lenToPoint > sqrt(pow(itepointx, 2.0) + pow(itepointy, 2.0) + pow(itepointz, 2.0)))
    {
      lenToPoint = sqrt(pow(itepointx, 2.0) + pow(itepointy, 2.0) + pow(itepointz, 2.0));
      cout << " i = " << i << endl;
      temp.push_back(i);
      p1 = temp.rbegin()[1];
    }
}
for(int i = 0; i < cloud->points.size()/2; i++)
{
  double itepointx = cloud->points[i].x - cloud->points[lastindex].x;
  double itepointy = cloud->points[i].y - cloud->points[lastindex].y;
  double itepointz = cloud->points[i].z - cloud->points[lastindex].z;

  if(lenToPoint1 > sqrt(pow(itepointx, 2.0) + pow(itepointy, 2.0) + pow(itepointz, 2.0)))
    {
      lenToPoint1 = sqrt(pow(itepointx, 2.0) + pow(itepointy, 2.0) + pow(itepointz, 2.0));
      cout << " i1 = " << i << endl;
      p2 = i;
}
}
return make_tuple(p1, p2);

}


int main(int argc, char** argv)
{
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr plane_out(new pcl::PointCloud<pcl::PointXYZ>);

pcl::io::loadPCDFile <pcl::PointXYZ>("concaveboi.pcd", *cloud);

int point1;
int point2;
//Replace 1 and 94 with the variables which hold the index values for the start and end of the longest line.
//idx1 and idx2
tie(point1, point2) = eighteen(cloud, 1, 94);
cout << "First point = " << point1 << " Second point = " << point2 << endl;
int lastindex = cloud->points.size()-1;
cout << lastindex << endl;
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  pub = nh.advertise<std_msgs::Int32MultiArray>("array", 1000);
  ros::Rate loop_rate(10);
  // Create a ROS subscriber for the input point cloud
  //ros::Subscriber sub = nh.subscribe ("legfromline.pcd", 1, callBack);
  //ros::spin 
while(ros::ok())
{



/*
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.setBackgroundColor(0.0, 0.0, 0.0);
  viewer.addPointCloud<pcl::PointXYZ>(cloud, "sample plane two");
  viewer.addCoordinateSystem(0.1);
  viewer.addLine(cloud->points[point1], cloud->points[lastindex], 0, 0, 1, "t");
  viewer.addLine(cloud->points[point2], cloud->points[lastindex], 0, 1, 0, "rt");

  while (!viewer.wasStopped())
  {
    viewer.spinOnce();
  }
*/

  std_msgs::Int32MultiArray array;
  //ROS_INFO("%s", msg.data.c_str());
  array.data.clear();

  array.data.push_back(point1);
  array.data.push_back(point2);

  pub.publish(array);

  ros::spinOnce();
  loop_rate.sleep();
  //++count;
}
  return 0;
}
