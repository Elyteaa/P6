#include <pcl/range_image/range_image.h>
#include <math.h>
#include <vector>
#include <algorithm>
//#include <ctime>
//#include <cstdlib>
#include <Eigen/Dense>
#include <Eigen/Geometry>
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

bool intersectingSegments(int longa, int longb, int shorta, int shortb, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    float A1 = (cloud->points[longa].y - cloud->points[longb].y) / (cloud->points[longa].x - cloud->points[longb].x);
    float A2 = (cloud->points[shorta].y - cloud->points[shortb].y) / (cloud->points[shorta].x - cloud->points[shortb].x);
    float b1 = cloud->points[longa].y - A1 * cloud->points[longa].x;
    float b2 = cloud->points[shorta].y - A2 * cloud->points[shorta].x;

    if (A1 == A2)
    {
        return false;
    } //parallel segments

    float Xa = (b2 - b1) / (A1 - A2);

    if (Xa < max(min(cloud->points[longa].x, cloud->points[longb].x),
        min(cloud->points[shorta].x, cloud->points[shortb].x)) ||
        (Xa > min(max(cloud->points[longa].x, cloud->points[longb].x),
        max(cloud->points[shorta].x, cloud->points[shortb].x))))
    {
        return false;
    } // Intersection out of bound
    else
    {
        return true;
    }
}

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
// Disregard lines that don't cross longest line. Make new vector w/ only relevant
std::vector<int> relevant;
for (int i = 0; i < iterations; i++)
{
    if (intersectingSegments(idx1, idx2, idx3[i], idx4[i], cloud))
    {
        relevant.push_back(i);
    }
    //cout << "Line: " << i << " does: [" << maybe << "] cross" << endl; 
}
for (int i = 0; i < relevant.size(); i++)
{
    cout << "relevant: [" << relevant[i] << "]" << endl;
}

// Calculate average distance between idx3 and idx4 pairs
float avg;
float sum = 0.0;
for (int i = 0; i < relevant.size(); i++)
{
    if (relevant[i])
    {
        /* code */
    }
    sum += shortest[i];
}
avg = sum/iterations;
cout << "Average length across: " << avg << endl;

////////////////////////////////////////////////////////////////////////////////
// Placing the cut based on: distance from tip, 0.18m

//Calculate angles 
Vector3f vec1, vec2;
vec1 << cloud->points[idx2].x - cloud->points[idx1].x,
        cloud->points[idx2].y - cloud->points[idx1].y,
        cloud->points[idx2].z - cloud->points[idx1].z;
cout << vec1 << endl;

vec1 = vec1.normalized();
cout << vec1 << endl;

float angle;
for (int i = 0; i < iterations; i++)
{
    vec2 << cloud->points[idx3[i]].x - cloud->points[idx4[i]].x,
            cloud->points[idx3[i]].y - cloud->points[idx4[i]].y,
            cloud->points[idx3[i]].z - cloud->points[idx4[i]].z;
    vec2 = vec2.normalized();
    
    float dotp = vec1.transpose() * vec2;
    angle = acos(dotp) * 180/3.14159265; //angle in deg

    cout << "angle " << i << " is: " << angle << endl;
}

//coordinates where the cut should be
float fix1, fix2;
float cutCoord[3] = {};
cutCoord[0] = cloud->points[idx2].x + vec1(0)*0;
cutCoord[1] = cloud->points[idx2].y + vec1(1)*0;
cutCoord[2] = cloud->points[idx2].z + vec1(2)*0;

//nearest point on one side
float cutWidth = 1.0;
for (int point = 0; point < cloud->points.size()/2; point++)
{
    xdist = cutCoord[0] - cloud->points[point].x;
    ydist = cutCoord[1] - cloud->points[point].y;
    zdist = cutCoord[2] - cloud->points[point].z;
    len = sqrt(pow(xdist, 2.0) + pow(ydist, 2.0) + pow(zdist, 2.0));

    if (len<cutWidth)
    {
        fix1 = point;
    }
}

//nearest point to that side
cutWidth = 1.0;
for (int point = 0; point < cloud->points.size(); point++)
{
    if (abs(point-fix1)>30)
    {
        xdist = cloud->points[fix1].x - cloud->points[point].x;
        ydist = cloud->points[fix1].y - cloud->points[point].y;
        zdist = cloud->points[fix1].z - cloud->points[point].z;
        len = sqrt(pow(xdist, 2.0) + pow(ydist, 2.0) + pow(zdist, 2.0));

        if (len<cutWidth)
            {
                cutWidth = len;
                fix2 = point;
            }
    }
}
cout << "Cut at fixed 18cm has width: " << cutWidth << endl;
cout << "Fixed cut has idxs: [" << fix1 << ", " << fix2 << "]" << endl;

///////////////////////////////////////////////////////////////////////////////
pcl::visualization::PCLVisualizer viewer("PCL Viewer");
viewer.setBackgroundColor (0, 0, 0);
viewer.addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");

for(int j = 0; j < iterations; j++) //idx-wise lines across the leg
{
      stringstream ss;
      ss << j;
      string str = ss.str();
      viewer.addLine(cloud->points[idx3[j]], cloud->points[idx4[j]], 1, 0, 0, str); //between points
}

viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud,normals);
viewer.addLine(cloud->points[idx1], cloud->points[idx2], 0, 1, 0, "q"); //Show longest line
viewer.addLine(cloud->points[fix1], cloud->points[fix2], 1, 1, 0, "fixedLength"); //18cm line

while(!viewer.wasStopped ())
{
    viewer.spinOnce ();
}
return 0;
}
