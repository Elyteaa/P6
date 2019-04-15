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

int main(int argc, char** argv)
{

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr plane (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr plane_out (new pcl::PointCloud<pcl::PointXYZ>);

pcl::io::loadPCDFile <pcl::PointXYZ> ("goodiethesecond.pcd", *cloud);

copyPointCloud(*cloud, *plane);

double bigx = 0;
double bigy = 0;
double smallx = 0;
double smally = 0;

for( int i = 0; i < plane->points.size(); i++)
{

	plane->points[i].z = 0;

}


for(double x = plane->points[0].x; x <= plane->points.size(); x++)
{

if((bigx < plane->points[x].x && bigy < plane->points[x].y) || (smallx > plane->points[x].x && smally > plane->points[x].y))
{
	bigx = plane->points[x].x;
	bigy = plane->points[x].y;

	smallx = plane->points[x].x;
	smally = plane->points[x].y;

	cout << "x = " << bigx << " y = " << bigy << endl;

	plane_out->push_back(plane->points[x]);
}
/*
if(bigx > plane->points[x].x && bigy > plane->points[x].y)
{
	bigx = plane->points[x].x;
	bigy = plane->points[x].y;

	cout << "x = " << smallx << " y = " << smally << endl;

	plane_out->push_back(plane->points[x]);
}

cout << "x = " << bigx << " y = " << bigy << endl;

*/

}

pcl::visualization::PCLVisualizer viewer("PCL Viewer");
viewer.setBackgroundColor (0.0, 0.0, 0.0);
//viewer.addPointCloud<pcl::PointXYZ> (plane, "sample cloud");
viewer.addPointCloud<pcl::PointXYZ> (plane_out, "sample plane");
viewer.addCoordinateSystem(0.1);

while(!viewer.wasStopped ())
{
	viewer.spinOnce ();
}


	return 0;
}
