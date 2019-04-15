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
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_return(new pcl::PointCloud<pcl::PointXYZ>);
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

	vector <int> value_list;
	//Scan x axis for max and min values for each point
	for (int i = minx; i < maxx - 1; i++)
	{
		//Iterate through the interval and choose 2 values
		for (int j; j < cloud->points.size(); i++)
		{
			//Save the points, located in the interval to a vector
			if (cloud->points[j].x >= )



			//Pick max and min value points and save them to the return cloud
		}

	}

	//Scan y axis for max and min values for each point


	return cloud_return;
}

int main(int argc, char** argv)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr plane_out(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::io::loadPCDFile <pcl::PointXYZ>("goodiethesecond.pcd", *cloud);

	copyPointCloud(*cloud, *plane);

	

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
