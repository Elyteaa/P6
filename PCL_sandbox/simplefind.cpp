#include <pcl/range_image/range_image.h>
#include <math.h>
//#include <vector>
//#include <ctime>
//#include <cstdlib>
#include <Eigen/Dense>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d.h>
#include <iostream>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace std;
using namespace Eigen;

int main (int argc, char** argv) {
srand (static_cast <unsigned> (time(0)));
// load point cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

pcl::io::loadPCDFile ("concaveboi.pcd", *cloud); //prev: hulltest.pdc
cout << "Point cloud size: " << cloud->points.size() << endl;

pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
ne.setViewPoint (1, 2, 4); //Set initial viewpoint
ne.setInputCloud(cloud);

pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
ne.setSearchMethod (tree);

// Output datasets
pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

// Use all neighbors in a sphere of radius 3cm
ne.setRadiusSearch (0.03);

// Compute the features
ne.compute(*normals);

pcl::ModelCoefficients::Ptr line_coefficients(new pcl::ModelCoefficients);
pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

///////////////////////////////////////////////////////////////////////////
// Try out different pairs, looking for the longest distance between them
const int size = cloud->points.size();
float longest = 0.0;
float len = 0.0;
float xdist = 0.0;
float ydist = 0.0;
float zdist = 0.0;
int idx1 = 0;
int idx2 = 0;

for (int i = 0; i < cloud->points.size()/2; i++)
{
    for (int j = 0; j < cloud->points.size(); j++)
    {
        if (abs(i-j)>cloud->points.size()/5) //exclude immediate neighbors
        {
            xdist = cloud->points[i].x - cloud->points[j].x;
            ydist = cloud->points[i].y - cloud->points[j].y;
            zdist = cloud->points[i].z - cloud->points[j].z;
            len = sqrt(pow(xdist, 2.0) + pow(ydist, 2.0) + pow(zdist, 2.0));

            if (len>longest)
            {
                longest = len;
                idx1 = i;
                idx2 = j;
            }
        }
    }
}

cout << "The longest line found goes between points [" << idx1 << "," << idx2
     << "] and has length: " << longest << endl;

/////////////////////////////////////////////////////////////////////////////////
// Placing the cut based on: thickness increase. Index increment
const int gran = 1; //granularity, how many indices are skipped per jump
const int iterations = cloud->points.size()/(gran*2);
float shortest[iterations] = {1.0};
int idx3[iterations] = {}; //nearest point to a randomized coord
int idx4[iterations] = {}; //nearest point opposite from idx3

// populate idx3 with indices starting from idx1
for (int i = 0; i < iterations; i++)
{
    if (idx1 + i*gran > cloud->points.size()-1)
    {
        idx3[i] = idx1 + i*gran - cloud->points.size();
    }
    else
    {
        idx3[i] = idx1 + i*gran;
    }
}

// find idx4 for each idx3
for (int i = 0; i < iterations; i++)
{
    if (abs(idx3[i] - idx1)<20 || idx3[i]<10 && abs(cloud->points.size() - idx1)<10)
    {//if nearest point is close to the tip
        if (idx3[i]>idx1) //if nearest point is to the right
        {
            if ((idx1-idx3[i])<0) //if the tip is close to idx(0)
            {
                  idx4[i] = cloud->points.size() - (idx3[i] - idx1);
            }
            else
            {
                  idx4[i] = idx1 - idx3[i];
            }
        }
        else //if nearest is to the left
        {
            if ((idx1 + idx3[i]) >= cloud->points.size()) //if the tip is close to idx(0)
            {
                idx4[i] = (idx1 - idx3[i]) - (cloud->points.size() - idx3[i]);
            }
            else
            {
                idx4[i] = idx1 + idx3[i];
            }
        }
        xdist = cloud->points[idx3[i]].x - cloud->points[idx4[i]].x;
        ydist = cloud->points[idx3[i]].y - cloud->points[idx4[i]].y;
        zdist = cloud->points[idx3[i]].z - cloud->points[idx4[i]].z;
        shortest[i] = sqrt(pow(xdist, 2.0) + pow(ydist, 2.0) + pow(zdist, 2.0));
    }
    else //if nearest point is not close to the tip
    {
        shortest[i] = 1.0;
        for (int point = 0; point < cloud->points.size(); point++)
        {//search for nearest opposite
            if (abs(point-idx3[i])>30)//exclude immediate neighbors
            {
                xdist = cloud->points[idx3[i]].x - cloud->points[point].x;
                ydist = cloud->points[idx3[i]].y - cloud->points[point].y;
                zdist = cloud->points[idx3[i]].z - cloud->points[point].z;
                len = sqrt(pow(xdist, 2.0) + pow(ydist, 2.0) + pow(zdist, 2.0));

                if (len<shortest[i])
                {
                    shortest[i] = len;
                    idx4[i] = point;
                }
            }
        }
    }
}

// Disregard lines that don't cross the middle
/////////////////////////////////////////////////////
//  Here we need to check if the lines cross the middle
/////////////////////////////////////////////////////





//Matrix<float, 1, 2> m1;
Vector3f m1, m2;
m2 <<   cloud->points[idx2].x - cloud->points[idx1].x,
        cloud->points[idx2].y - cloud->points[idx1].y,
        cloud->points[idx2].z - cloud->points[idx1].z;



for (int i = 0; i < iterations; i++)
{
    /*
    across[0] = cloud->points[idx3[i]].x - cloud->points[idx4[i]].x;
    across[1] = cloud->points[idx3[i]].x - cloud->points[idx4[i]].x;
    across[2] = cloud->points[idx3[i]].x - cloud->points[idx4[i]].x;*/

    m2 <<   cloud->points[idx3[i]].x - cloud->points[idx4[i]].x,
            cloud->points[idx3[i]].y - cloud->points[idx4[i]].y,
            cloud->points[idx3[i]].z - cloud->points[idx4[i]].z;

    //MatrixXf m3 = m1.transpose() * m2;
    float m4 = m1.transpose() * m2;

    if (abs(m4)<0.01)
    {
        cout << "Angle for idx: " << i << "is small" << endl;
    }
}


/*
m2 <<   cloud->points[idx3[3]].x - cloud->points[idx4[3]].x,
            cloud->points[idx3[3]].x - cloud->points[idx4[3]].x,
            cloud->points[idx3[3]].x - cloud->points[idx4[3]].x;
            */

//MatrixXf m3 = m1.transpose() * m2;


cout << "m1: " << m1 << endl;
cout << "m2: " << m2 << endl;
//cout << "m3 : " << m3 << endl;



// Calculate average distance between idx3 and idx4 pairs
float avg;
float sum = 0.0;
for (int i = 0; i < iterations; i++)
{
    sum += shortest[i];
}
avg = sum/iterations;
cout << "avg length: " << avg << endl;

////////////////////////////////////////////////////////////////////////////////
// Placing the cut based on: distance from tip, 180mm



/*
for(int j = 0; j < iterations; j++)
{
      cout << "idx3[" << j << "]: " << idx3[j] << endl;
      cout << "idx4[" << j << "]: " << idx4[j] << endl;
}

for(int j = 0; j < iterations; j++)
{
      cout << "shortest[" << j << "]: " << shortest[j] << endl;
}
*/

///////////////////////////////////////////////////////////////////////////////
pcl::visualization::PCLVisualizer viewer("PCL Viewer");
viewer.setBackgroundColor (0, 0, 0);
viewer.addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");

for(int j = 0; j < iterations; j++)
{
      stringstream ss;
      ss << j;
      string str = ss.str();
      viewer.addLine(cloud->points[idx3[j]], cloud->points[idx4[j]], 1, 0, 0, str); //between points
}

viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud,normals);
viewer.addLine(cloud->points[idx1], cloud->points[idx2], 0, 1, 0, "q"); //Show longest line

while(!viewer.wasStopped ())
{
    viewer.spinOnce ();
}
return 0;
}
