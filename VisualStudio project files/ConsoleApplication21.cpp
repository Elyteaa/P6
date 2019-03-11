#include "stdafx.h"
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
//#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

void showCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr showThis)
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Viewer"));
	//viewer.showCloud(showThis);
	viewer->setBackgroundColor(0,0,0);
	viewer->removeAllShapes();
	viewer->removeAllPointClouds();

	viewer->addPointCloud<pcl::PointXYZ>(showThis, "Uploaded1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Uploaded1");

	viewer->spinOnce(100);
	/*
	while (!viewer.wasStopped())
	{
	}
	*/
}

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPLYFile<pcl::PointXYZ>("legcloud.ply", *cloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file \n");
		return (-1);
	}

	while (true) {
		showCloud(cloud);
	}
	//showCloud(cloud);
	std::cout << "Loaded "
		<< cloud->width * cloud->height
		<< " data points from cat.pcd with the following fields: "
		<< std::endl;
	for (size_t i = 0; i < cloud->points.size(); ++i)
		std::cout << "    " << cloud->points[i].x
		<< " " << cloud->points[i].y
		<< " " << cloud->points[i].z << std::endl;

	return (0);
}