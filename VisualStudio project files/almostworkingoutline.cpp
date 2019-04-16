#include "stdafx.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/io.h>
#include <vector>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/project_inliers.h>
#include <pcl/filters/passthrough.h>

using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr quarterLiers (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_return(new pcl::PointCloud<pcl::PointXYZ>);
	double bigx = 0;
	double bigy = 0;

	for (int i = 0; i < cloud->points.size(); i++)
	{
		cloud->points[i].z = 0;
	}

	for (int x = 0; x < cloud->points.size(); x++)
	{
		if ((bigx < cloud->points[x].x && bigy > cloud->points[x].y) || (bigx > cloud->points[x].x && bigy < cloud->points[x].y))
		{
			cout << "bigx = " << bigx << " bigy = " << bigy << endl;
			bigx = cloud->points[x].x;
			bigy = cloud->points[x].y;

			cout << "x = " << cloud->points[x].x << " y = " << cloud->points[x].y << endl;

			cloud_return->push_back(cloud->points[x]);
		}
	}

	return cloud_return;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr scanAxis (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_return (new pcl::PointCloud<pcl::PointXYZ>);
	double minx = cloud->points[0].x;
	double miny = cloud->points[0].y;
	double maxx = minx;
	double maxy = miny;

	for (int i = 0; i < cloud->points.size(); i++)
	{
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
	for (int i = minx; i < maxx - 1; i++)
	{
		vector <pcl::PointXYZ> value_listx;
		//Iterate through the interval and choose 2 values
		for (int j = 0; j < cloud->points.size(); j++)
		{
			//Save the points, located in the interval to a vector
			if ((cloud->points[j].x >= minx) && (cloud->points[j].x < minx + 1))
			{
				value_listx.push_back(cloud->points[j]);
			}
		}

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
		cloud_return->push_back(value_listx.at(i_miny));
		cloud_return->push_back(value_listx.at(i_maxy));
	}

	//Scan y axis for max and min values for each point
	for (int i = miny; i < maxy - 1; i++)
	{
		vector <pcl::PointXYZ> value_listy;
		//Iterate through the interval and choose 2 values
		for (int j = 0; j < cloud->points.size(); j++)
		{
			//Save the points, located in the interval to a vector
			if ((cloud->points[j].y >= miny) && (cloud->points[j].y < miny + 1))
			{
				value_listy.push_back(cloud->points[j]);
			}
		}

		//Pick max and min value points and save them to the return cloud
		double i_minx = 0;
		double i_maxx = 0;

		for (int k = 1; k < value_listy.size(); k++)
		{
			if (value_listy.at(i_minx).x > value_listy.at(k).x) { i_minx = k; }
			if (value_listy.at(i_maxx).x < value_listy.at(k).x) { i_maxx = k; }
		}

		//Pushback found points
		cloud_return->push_back(value_listy.at(i_minx));
		cloud_return->push_back(value_listy.at(i_maxx));
	}

	return cloud_return;
}

int main(int argc, char** argv)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr plane_out(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::io::loadPCDFile <pcl::PointXYZ>("goodiethesecond.pcd", *cloud);

	copyPointCloud(*cloud, *plane);

	plane_out = scanAxis(cloud);

	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	viewer.setBackgroundColor(0.0, 0.0, 0.0);
	//viewer.addPointCloud<pcl::PointXYZ> (plane, "sample cloud");
	viewer.addPointCloud<pcl::PointXYZ>(plane_out, "sample plane");
	viewer.addCoordinateSystem(0.1);

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}

	return 0;
}
