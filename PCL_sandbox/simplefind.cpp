#include <pcl/range_image/range_image.h>
#include <math.h>
#include <vector>
#include <algorithm>
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

class Pose {
private:
    float position[3];
    float orientation[3];

public:
    
    void set_values(){
        cout << "hello class" << endl;
    }
};

tuple<int, int> eighteen (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int start, int end)
{

double distx = cloud->points[start].x - cloud->points[end].x;
double disty = cloud->points[start].y - cloud->points[end].y;
double distz = cloud->points[start].z - cloud->points[end].z;
//Finding the length of the line
double len1 = sqrt(pow(distx, 2.0) + pow(disty, 2.0) + pow(distz, 2.0));

double difflen = 0.14/len1;

//Scaling the line to 0.18
len1 *= difflen;

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

    if(lenToPoint > sqrt(pow(itepointx, 2.0) + pow(itepointy, 2.0) + pow(itepointz, 2.0)))
    {
        lenToPoint = sqrt(pow(itepointx, 2.0) + pow(itepointy, 2.0) + pow(itepointz, 2.0));
        temp.push_back(i);
        p1 = temp.rbegin()[1];
    }
}
for(int i = 0; i < cloud->points.size(); i++)
{
    if (abs(p1-i)>30)//exclude immediate neighbors
    {
        double itepointx = cloud->points[i].x - cloud->points[p1].x;
        double itepointy = cloud->points[i].y - cloud->points[p1].y;
        double itepointz = cloud->points[i].z - cloud->points[p1].z;

        if(lenToPoint1 > sqrt(pow(itepointx, 2.0) + pow(itepointy, 2.0) + pow(itepointz, 2.0)))
        {
            lenToPoint1 = sqrt(pow(itepointx, 2.0) + pow(itepointy, 2.0) + pow(itepointz, 2.0));
            p2 = i;
        }
    }
}
return make_tuple(p1, p2);
}


bool startIsNarrower(float myArray[], int arraySize){
//Checks if the first half of an array has a smaller mean value than the 2nd half
float sum = 0.0, avg1, avg2;
    for (int i = 0; i < arraySize/2; i++)
    {
        sum += myArray[i];
    }
    avg1 = sum/arraySize/2;

    sum = 0.0;
    for (int i = arraySize/2; i < arraySize; i++)
    {
        sum += myArray[i];
    }
    cout << "hello function" << endl;

    avg2 = sum/arraySize/2;

    if (avg1<avg2)
    {
        return true;
    } else {
        return false;
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
float longest = 0.0; // longest line found
float len = 0.0;     // current length being evaluated
float xdist = 0.0;
float ydist = 0.0;   // distance between points in y-direction
float zdist = 0.0;
int idx1 = 0;        // one end of the line
int idx2 = 0;        // other end of the line

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
float shortest[iterations] = {1.0}; //distances between pairs of idx3[i] and idx4[i]
int idx3[iterations] = {};
int idx4[iterations] = {}; //nearest point opposite from an idx3

// populate idx3 with indices, starting from idx1
for (int i = 0; i < iterations; i++)
{
    if (idx1 + i*gran > cloud->points.size()-1)
    {// in case we would exceed size of point cloud
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
            if (((idx1-idx3[i])<10) || (cloud->points.size() - idx1)<10)
            {//if the tip is close to idx(0)
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
/////////////////////////////////////////////////////////////////////////////////
// Make sure that idx1 is the narrow end
if (startIsNarrower(shortest, iterations))
{
    cout << "idx1 is already in the narrow end of the leg" << endl;
} else {
    int temp = idx1; // switch them
    idx1 = idx2;
    idx2 = temp;
}
/////////////////////////////////////////////////////////////////////////////////
// Discriminate lines based on angle // Calculate angles
Vector3f vec1, vec2;
vec1 << cloud->points[idx2].x - cloud->points[idx1].x,
        cloud->points[idx2].y - cloud->points[idx1].y,
        cloud->points[idx2].z - cloud->points[idx1].z;
vec1 = vec1.normalized(); // vector version of the longest line (green)

std::vector<int> relVec; //only valid lines 
uint valCount = 0; //How many lines are valid
float angle[iterations] = {}; //Angles of the lines
for (int i = 0; i < iterations; i++)
{
    vec2 << cloud->points[idx3[i]].x - cloud->points[idx4[i]].x,
            cloud->points[idx3[i]].y - cloud->points[idx4[i]].y,
            cloud->points[idx3[i]].z - cloud->points[idx4[i]].z;
    vec2 = vec2.normalized(); //to ensure it has length 1
    
    float dotp = vec1.transpose() * vec2;
    angle[i] = acos(dotp) * 180/3.14159265; //angle in deg

    if (abs(angle[i] - 90) < 35) //accept a diff of up to x deg // WEIRD ERROR WITH LOW VALUE
    {
        relVec.push_back(i+1); //positive indicates valid line
        valCount++; // count valid lines
    }
    else
    {
        relVec.push_back(0); // 0 indicates invalid line
    }
    //cout << "angle " << i << " is: " << angle[i] << endl;
}

//Make new array with only valid lines
float relShort[valCount] = {}; //lengths of valid lines
uint relcount = 0;
for (int i = 0; i < iterations; i++)
{
    if (relVec[i]) //if this index's line is valid
    {
        relShort[relcount] = shortest[i];
        relcount++;
    }
}

for (int i = 0; i < valCount; ++i)
{
    cout << "relShort [" << i << "] has value: " << relShort[i] << endl;
}

///////////////////////////////////////////////////////////////////////////////
// Evaluate thickness //
// Find local minimum, starting from the middle
uint minIdx = valCount/2;
bool minFound = false;
uint minTries = 0;
uint check = valCount/12; //Check a nr of steps, proportional to size of array
int dir = 1; //is pos to ensure that it can move, if it starts in a local minimum
while(!minFound)
{
    if (relShort[minIdx]>relShort[minIdx-1])
    {
        minIdx--;
        dir = -1;
        cout << "minus" << endl;
    }
    else if (relShort[minIdx]>relShort[minIdx+1])
    {
        minIdx++;
        dir = 1;
        cout << "plus" << endl;
    }
    else if (relShort[minIdx]<relShort[minIdx+1] && relShort[minIdx]<relShort[minIdx-1])
    {
        cout << "Possible min at: " << minIdx << endl;
        for (int i = 1; i <= check; i++)
        {
            if (relShort[minIdx + i*dir] < relShort[minIdx])
            {
                minIdx = minIdx + i*dir;
                i = check;
            }
        }
        if (!dir)
        {
            minFound = true;
            cout << "Found min: [" << minIdx << "] with value: [" << relShort[minIdx] << "]" << endl;
        }
        if (minTries < 1)
        {
            dir = -dir;
            minTries++;
        }
        else
        {
            dir = 0;
        }
    }
    else 
    {
        cout << "Error in finding minimum!" << endl;
        break;
    }
}

// Go from minIdx back to real idx of local minimum
uint finIdx = 0;
for (int i = 0; i < iterations; i++)
{
    if (relVec[i])
    {
        finIdx++;
    }
    if (finIdx == minIdx + 1)
    {
        finIdx = i;
        i = iterations;
    }
}

cout << "Min has original idx: [" << finIdx << "] with value: [" << shortest[finIdx] << "]" << endl;

////////////////////////////////////////////////////////////////////////////////
// Placing the cut based on: distance from tip, 0.18m
//coordinates where the cut should be
int point1;
int point2;
//Replace 1 and 94 with the variables which hold the index values for the start and end of the longest line.
//idx1 and idx2
tie(point1, point2) = eighteen(cloud, idx1, idx2);
cout << "First point = " << point1 << " Second point = " << point2 << endl;
int lastindex = cloud->points.size()-1;
cout << lastindex << endl;


/////////////////////////////////////////////////////////////////////////////////
// Combine the two estimates




///////////////////////////////////////////////////////////////////////////////
// Output concluded end effector pose



Pose myPose;
myPose.set_values();


///////////////////////////////////////////////////////////////////////////////
// Viewer //
pcl::visualization::PCLVisualizer viewer("PCL Viewer");
viewer.setBackgroundColor (0, 0, 0);
viewer.addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");

for(int j = 0; j < iterations; j++) //idx-wise lines across the leg
{
    stringstream ss;
    ss << j;
    string str = ss.str();
    if (relVec[j])
    {
        if (j == finIdx) //concluded index
        {
            viewer.addLine(cloud->points[idx3[j]], cloud->points[idx4[j]], 1, 1, 0, str); //concluded line
        }
        else
        {
            viewer.addLine(cloud->points[idx3[j]], cloud->points[idx4[j]], 1, 0, 0, str); //between points
        }
    }
}

viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud,normals);
viewer.addLine(cloud->points[idx1], cloud->points[idx2], 0, 1, 0, "q"); //Show longest line
viewer.addLine(cloud->points[point1], cloud->points[point2], 0, 1, 0, "t"); //18cm line

while(!viewer.wasStopped ())
{
    viewer.spinOnce ();
}
return 0;
}
