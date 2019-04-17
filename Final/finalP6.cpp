#include "stdafx.h"
#include <iostream>
#include <vector>
#include <ros/ros.h>
//#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
//#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <boost/thread/thread.hpp>

using namespace std;

ros::Publisher pub;

pcl::PointCloud<pcl::PointXYZ>::Ptr scanAxis (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	const int SCALE_SIZE = 1000;
	const int INTERVAL_SIZE = 5;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_return (new pcl::PointCloud<pcl::PointXYZ>);

	float minx = cloud->points[0].x;
	float miny = cloud->points[0].y;
	float maxx = minx;
	float maxy = miny;

	for (int i = 0; i < cloud->points.size(); i++)
	{
		cloud->points[i].x *= SCALE_SIZE;
		cloud->points[i].y *= SCALE_SIZE;
		cloud->points[i].z = 0;
	}

	//Find min x and y values
	for (int i = 1; i < cloud->size(); i++)
	{
		if (cloud->points[i].x < minx) { minx = cloud->points[i].x; }
		else if (cloud->points[i].x > maxx) { maxx = cloud->points[i].x; };
		if (cloud->points[i].y < miny) { miny = cloud->points[i].y; }
		else if (cloud->points[i].y > maxy) { maxy = cloud->points[i].y; };
	}

	//Scan x axis for max and min values for each point
	for (float i = minx; i < maxx; i+= INTERVAL_SIZE)
	{
		vector <pcl::PointXYZ> value_listx;
		//Iterate through the interval and choose 2 values
		for (int j = 0; j < cloud->points.size(); j++)
		{
			//Save the points, located in the interval to a vector
			if ((cloud->points[j].x >= i) && (cloud->points[j].x < i + INTERVAL_SIZE))
			{
				value_listx.push_back(cloud->points[j]);
			}
		}

		//Pick max and min value points and save them to the return cloud
		if (value_listx.size() == 1)
		{
			value_listx.at(0).x /= SCALE_SIZE;
			value_listx.at(0).y /= SCALE_SIZE;
			cloud_return->push_back(value_listx.at(0));
		}
		else if (value_listx.size() > 1)
		{
			int i_min = 0;
			int i_max = 0;

			for (int k = 1; k < value_listx.size(); k++)
			{
				if (value_listx.at(i_min).y > value_listx.at(k).y) { i_min = k; }
				if (value_listx.at(i_max).y < value_listx.at(k).y) { i_max = k; }
			}

			//Pushback found points
			value_listx.at(i_min).x /= SCALE_SIZE;
			value_listx.at(i_min).y /= SCALE_SIZE;
			value_listx.at(i_max).x /= SCALE_SIZE;
			value_listx.at(i_max).y /= SCALE_SIZE;
			cloud_return->push_back(value_listx.at(i_min));
			cloud_return->push_back(value_listx.at(i_max));
		}
		
	}

	//Scan y axis for max and min values for each point
	for (int i = miny; i < maxy; i += INTERVAL_SIZE)
	{
		vector <pcl::PointXYZ> value_listy;
		//Iterate through the interval and choose 2 values
		for (int j = 0; j < cloud->points.size(); j++)
		{
			//Save the points, located in the interval to a vector
			if ((cloud->points[j].y >= i) && (cloud->points[j].y < i + INTERVAL_SIZE))
			{
				value_listy.push_back(cloud->points[j]);
			}
		}

		//Pick max and min value points and save them to the return cloud
		if (value_listy.size() == 1)
		{
			value_listy.at(0).x /= SCALE_SIZE;
			value_listy.at(0).y /= SCALE_SIZE;
			cloud_return->push_back(value_listy.at(0));
		}
		else if (value_listy.size() > 1)
		{
			int i_min = 0;
			int i_max = 0;

			for (int k = 1; k < value_listy.size(); k++)
			{
				if (value_listy.at(i_min).y > value_listy.at(k).y) { i_min = k; }
				if (value_listy.at(i_max).y < value_listy.at(k).y) { i_max = k; }
			}

			//Pushback found points
			value_listy.at(i_min).x /= SCALE_SIZE;
			value_listy.at(i_min).y /= SCALE_SIZE;
			value_listy.at(i_max).x /= SCALE_SIZE;
			value_listy.at(i_max).y /= SCALE_SIZE;
			cloud_return->push_back(value_listy.at(i_min));
			cloud_return->push_back(value_listy.at(i_max));
		}
	}

	return cloud_return;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr smooth (pcl::PointCloud<pcl::PointXYZ>::Ptr plane_out)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);

	double minx = plane_out->points[0].x;
	double miny = plane_out->points[0].y;
	double maxx = minx;
	double maxy = miny;

	//Find min x and y values
	for (int i = 1; i < plane_out->size(); i++)
	{
		if (plane_out->points[i].x < minx) { minx = plane_out->points[i].x; }
		else if (plane_out->points[i].x > maxx) { maxx = plane_out->points[i].x; };
		if (plane_out->points[i].y < miny) { miny = plane_out->points[i].y; }
		else if (plane_out->points[i].y > maxy) { maxy = plane_out->points[i].y; };

		//cout << "minx = " << minx << " miny = " << miny << " maxx = " << maxx << " maxy = " << maxy << endl;
	}

	//Scan x axis for max and min values for each point
	//for (int i = minx; i < maxx - 1; i++)
	//{
		vector <pcl::PointXYZ> value_listx;
		//cout << i << endl;
		//Iterate through the interval and choose 2 values
		for (int j = 0; j < plane_out->points.size(); j++)
		{
			//Save the points, located in the interval to a vector
			if ((plane_out->points[j].x >= minx) && (plane_out->points[j].x <= minx + 1))
			{
				value_listx.push_back(plane_out->points[j]);
				//cout << value_listx.size() << endl;
			}
		//}

		cout << value_listx.size() << endl;

		//Pick max and min value points and save them to the return cloud
		double i_miny = 0;
		double i_maxy = 0;

		for (int k = 1; k < value_listx.size(); k++)
		{
			if (value_listx.at(i_miny).y > value_listx.at(k).y) { i_miny = k; }
			if (value_listx.at(i_maxy).y < value_listx.at(k).y) { i_maxy = k; }
		}

		//Pushback found points
		cloud_final->push_back(value_listx.at(i_miny));
		cloud_final->push_back(value_listx.at(i_maxy));
	}

	//Scan y axis for max and min values for each point
	//for (int i = miny; i < maxy - 1; i++)
	//{
	
		vector <pcl::PointXYZ> value_listy;
		//Iterate through the interval and choose 2 values
		for (int j = 0; j < plane_out->points.size(); j++)
		{
			//Save the points, located in the interval to a vector
			if ((plane_out->points[j].y >= miny) && (plane_out->points[j].y <= miny + 1))
			{
				value_listy.push_back(plane_out->points[j]);
			}
		//}

		//Pick max and min value points and save them to the return cloud
		double i_minx = 0;
		double i_maxx = 0;

		for (int k = 1; k < value_listy.size(); k++)
		{
			if (value_listy.at(i_minx).x > value_listy.at(k).x) { i_minx = k; }
			if (value_listy.at(i_maxx).x < value_listy.at(k).x) { i_maxx = k; }
		}

		//Pushback found points
		cloud_final->push_back(value_listy.at(i_minx));
		cloud_final->push_back(value_listy.at(i_maxx));
	}

	return cloud_final;
}

void callBack(const sensor_msgs::PointCloud2ConstPtr& input)
{
	pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2), cloud_filtered_blob(new pcl::PCLPointCloud2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>), cloud_p(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>), plane_out(new pcl::PointCloud<pcl::PointXYZ>), plane_please_work(new pcl::PointCloud<pcl::PointXYZ>);

	//Using the PointCloud2 data from the ROS topic
	pcl_conversions::toPCL(*input, *cloud_blob);

	// Convert to the templated PointCloud
	pcl::fromPCLPointCloud2(*cloud_blob, *cloud_filtered);

	//See if it can be done with 1-2 clouds
	// Create the filtering object
 	pcl::PassThrough<pcl::PointXYZ> pass;
 	pass.setInputCloud(cloud);
	pass.setFilterFieldName ("z");
 	pass.setFilterLimits(0.30, 1.03);
  	pass.filter(*cloud_filtered);
  
	// pcl::PassThrough<pcl::PointXYZ> pass1;
  	pass.setInputCloud(cloud_filtered);
  	pass.setFilterFieldName ("x");
  	pass.setFilterLimits(-0.30, 0.20);
	pass.filter(*final_cloud);

	for (int i = 0; i < cloud->points.size(); i++)
	{
		final_cloud->points[i].z = 0;
	}

	plane_out = scanAxis(final_cloud);
	plane_please_work = smooth(final_cloud);
}

int main(int argc, char** argv)
{
	// Initialize ROS
	ros::init(argc, argv, "my_pcl_tutorial");
	ros::NodeHandle nh;

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe("/kinect2/sd/points", 1, callBack);

	// Create a ROS publisher for the output model coefficients
	pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

	// Spin
	ros::spin();
}