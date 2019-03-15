//VisualStudio stuff
#include "stdafx.h"
//Our functions
#include "PointCloudProcessing.h"
//Needed stuff
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <boost/make_shared.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <iostream>
using namespace std;

void showCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr showThis)
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Viewer"));
	//viewer.showCloud(showThis);
	viewer->setBackgroundColor(0,0,0);
	viewer->removeAllShapes();
	viewer->removeAllPointClouds();

	viewer->addPointCloud<pcl::PointXYZ>(showThis, "Uploaded1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Uploaded1");

	//viewer->spinOnce(100);
	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}
}

/*void VoxelGrid(pcl::PCLPointCloud2::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr showThis;
	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(0.01f, 0.01f, 0.01f);
	sor.filter(*cloud);
	pcl::fromPCLPointCloud2(*cloud, *showThis);

	showCloud(showThis);
}*/

void showNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr showThis, pcl::PointCloud<pcl::Normal>::Ptr showThisNormal)
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("NormalViewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->removeAllShapes();
	viewer->removeAllPointClouds();

	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(showThis, showThisNormal);
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Uploaded1");

	//viewer->spinOnce(100);
	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}
}

void findNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);

	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

	//ne.setKSearch(4);

	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch(0.03);

	// Compute the features
	ne.compute(*cloud_normals);

	// cloud_normals->points.size () should have the same size as the input cloud->points.size ()*

	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	viewer.setBackgroundColor(0.0, 0.0, 0.5);
	viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normals);

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}

	//boost::shared_ptr<pcl::PointCloud<pcl::Normal>> cloud_normals_shared = boost::make_shared<pcl::PointCloud<pcl::Normal>>(*cloud_normals);

	//showCloud(cloud);
	//showNormals(cloud, cloud_normals_shared);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr segmentTheImage(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	
	// Optional
	seg.setOptimizeCoefficients(true);
	
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(0.01);

	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZ> extract;

	int i = 0, nr_points = (int)cloud->points.size();
	// While 30% of the original cloud is still there
	while (cloud->points.size() > 0.3 * nr_points)
	{
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud(cloud);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0)
		{
			std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// Extract the inliers
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*cloud_p);
		std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

		// Create the filtering object
		extract.setNegative(true);
		extract.filter(*cloud_f);
		cloud.swap(cloud_f);
		i++;
	}

	cout << "Something has been done" << endl;
	showCloud(cloud);*/

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	// Create the filtering object
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.6, 0.9);
	pass.filter(*cloud_filtered);

	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	viewer.setBackgroundColor(0.0, 0.0, 0);
	viewer.addPointCloud<pcl::PointXYZ>(cloud_filtered, "sample cloud");

	return (cloud_filtered);
}

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_segmented(new pcl::PointCloud<pcl::PointXYZ>);;

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("nice_leg3.pcd", *cloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file \n");
		return (-1);
	}

	std::cout << "Loaded "
		<< cloud->width << " " << cloud->height << " "
		//<< cloud->at(200,200).z
		/*
		<< " data points from cat.pcd with the following fields: "
		<< std::endl;
		for (size_t i = 0; i < cloud->points.size(); ++i)
		std::cout << "    " << cloud->points[i].x
		<< " " << cloud->points[i].y
		<< " " << cloud->points[i].z
		*/
		<< std::endl;

	cloud_segmented = segmentTheImage(cloud);
	findNormals(cloud_segmented);

	showCloud(cloud_segmented);

	//findNormals(cloud);

	//show cloud
	//showCloud(cloud);

	return (0);
}