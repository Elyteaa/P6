//#include "stdafx.h"
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
		if ((bigx < cloud->points[x].x && bigy < cloud->points[x].y) || (bigx > cloud->points[x].x && bigy > cloud->points[x].y))
		{
			bigx = cloud->points[x].x;
			bigy = cloud->points[x].y;

			cloud_return->push_back(cloud->points[x]);
		}
	}

	return cloud_return;
}

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

pcl::PointCloud<pcl::PointXYZ>::Ptr smooth (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);

	double minx = cloud->points[0].x;
	double miny = cloud->points[0].y;
	double maxx = minx;
	double maxy = miny;

	//Find min x and y values
	for (int i = 1; i < cloud->size(); i++)
	{
		if (cloud->points[i].x < minx) { minx = cloud->points[i].x; }
		else if (cloud->points[i].x > maxx) { maxx = cloud->points[i].x; };
		if (cloud->points[i].y < miny) { miny = cloud->points[i].y; }
		else if (cloud->points[i].y > maxy) { maxy = cloud->points[i].y; };

		//cout << "minx = " << minx << " miny = " << miny << " maxx = " << maxx << " maxy = " << maxy << endl;
	}

	//Scan x axis for max and min values for each point
	//for (int i = minx; i < maxx - 1; i++)
	//{
		vector <pcl::PointXYZ> value_listx;
		//cout << i << endl;
		//Iterate through the interval and choose 2 values
		for (int j = 0; j < cloud->points.size(); j++)
		{
			//Save the points, located in the interval to a vector
			if ((cloud->points[j].x >= minx) && (cloud->points[j].x <= minx + 1))
			{
				value_listx.push_back(cloud->points[j]);
				//cout << value_listx.size() << endl;
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
		cloud_final->push_back(value_listx.at(i_miny));
		cloud_final->push_back(value_listx.at(i_maxy));
	//}

	//Scan y axis for max and min values for each point
	//for (int i = miny; i < maxy - 1; i++)
	//{
	
		vector <pcl::PointXYZ> value_listy;
		//Iterate through the interval and choose 2 values
		for (int j = 0; j < cloud->points.size(); j++)
		{
			//Save the points, located in the interval to a vector
			if ((cloud->points[j].y >= miny) && (cloud->points[j].y <= miny + 1))
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
		cloud_final->push_back(value_listy.at(i_minx));
		cloud_final->push_back(value_listy.at(i_maxx));
	//}

	return cloud_final;
}

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr plane_out(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_please_work(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::io::loadPCDFile <pcl::PointXYZ>("goodiethesecond.pcd", *cloud);

	copyPointCloud(*cloud, *plane);

	/*int temp_i = cloud->points.size() - 1;
	for (int i = 0; i < 20000; i++)
	{
		cout << i << endl;
		pcl::PointXYZ temp;
		temp.x = (cloud->points[i].x + cloud->points[i + 1].x) / 2;
		temp.y = (cloud->points[i].y + cloud->points[i + 1].y) / 2;
		temp.z = 0;
		cloud->push_back(temp);
	}*/

	plane = quarterLiers(cloud);
	plane_out = scanAxis(cloud);
	copyPointCloud(*plane_out, *plane_please_work);
	plane_please_work = smooth(cloud);


	cout << "returned cloud size " << plane_out->points.size() << endl;
	
	/*for (int i = 0; i < plane->points.size(); i++)
	{
		for  (int j = 0; j < plane_out->points.size(); j++)
		{
			if ((plane->points[i].x == plane_out->points[j].x) && (plane->points[i].y == plane_out->points[j].y))// {
				//true;
			//} 
			{	plane_out->points.erase(plane_out->points.begin() + j);	}
		}
	}*/
	
	//cout << "size: " << plane_out->points.size() << endl;
//pcl::PCDWriter writer;
//  writer.write ("emptyshell.pcd", *plane_out, false);
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	viewer.setBackgroundColor(0.0, 0.0, 0.0);
	//viewer.addPointCloud<pcl::PointXYZ> (plane, "sample cloud");
	//viewer.addPointCloud<pcl::PointXYZ>(plane, "sample plane");
	viewer.addPointCloud<pcl::PointXYZ>(plane_please_work, "sample plane two");
	/*viewer.addLine(plane_out->points[0], plane_out->points[1], 1, 0, 0, "uhhhh");
	viewer.addLine(plane_out->points[1], plane_out->points[2], 0, 1, 0, "palne");
	viewer.addLine(plane_out->points[2], plane_out->points[3], 0, 0, 1, "dfka");
	viewer.addLine(plane_out->points[3], plane_out->points[4], 1, 1, 1, "ol");*/
	viewer.addCoordinateSystem(0.1);

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}

	return 0;
}
