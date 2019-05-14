//PCL
#include <pcl/range_image/range_image.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/passthrough.h>

//c++
#include <math.h>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

//PCL ROS
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>

//ROS
#include "std_msgs/Float32MultiArray.h"
//Might not need these
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

using namespace std;

ros::Publisher pub;
// Refine estimate combination
// Check tool orientation
// Implement in ROS
// Integrate everything

/*
Fig 6.1 good
- what about communicating with the FANUC 710ic?
- ros has JointState and Pose msg types, why not use those?
- discriminate line?
- You have almost no comments in you code. Find some guidelines on writing useful comments. I can recommend writing comments before writing the code itself.
- //A "setter" is usually a very simple piece of code, name that function something more telling.
- //Saying normalize to run smoothly is not enough
- make a drawing showing the various named points and vectors on the leg. Naming could be improved a lot.
*/

float Dist(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int index1, int index2);
float Dist(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int index1);

class Pose {
private:
    float pose[6];

public:
void calculatePose(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int start1, int end1, int start2, int end2){
// make the point indicating position
pose[0] = cloud->points[end1].x + (cloud->points[start1].x - cloud->points[end1].x)/2;
pose[1] = cloud->points[end1].y + (cloud->points[start1].y - cloud->points[end1].y)/2;
pose[2] = cloud->points[end1].z + (cloud->points[start1].z - cloud->points[end1].z)/2;
//define vector between sides of leg
Eigen::Vector3f across, along, rotvec, planevec, robvec;
across << cloud->points[end1].x - cloud->points[start1].x,
          cloud->points[end1].y - cloud->points[start1].y,
          cloud->points[end1].z - cloud->points[start1].z;
across = across.normalized();

//define vector that is the longest line
along << cloud->points[end2].x - cloud->points[start2].x,
         cloud->points[end2].y - cloud->points[start2].y,
         cloud->points[end2].z - cloud->points[start2].z;
along = along.normalized();

rotvec = across.cross(along); //vector to be rotated around the plane vector
rotvec = rotvec.normalized();
planevec = across.cross(rotvec); //vector to define plane
planevec = planevec.normalized();
robvec << pose[0], pose[1], pose[2]; // unit vector from the leg to the robot (or reverse that?)

float angle[36] = {}; //contains found angles. mb not necessary
float smallAngle = 360; //smallest found angle. high initial value
float angleInc = 10*(3.14159265/180); //Angle increment in radians, 10 deg
int vecIdx; //which vector fits best
float dotp; //dotproduct

for (int i = 0; i < 36; i++)
{ //test which vector fits best
    rotvec = rotvec*cos(angleInc) + (planevec.cross(rotvec))*sin(angleInc);// + planevec*(planevec*rotvec)*(1-cos(10); //Rodrigue's formula, abridged
    dotp = rotvec.transpose() * robvec; // dot product with robot vector
    angle[i] = acos(dotp) * 180/3.14159265; //angle in deg
    if (angle[i]<smallAngle)
    {
        smallAngle = angle[i];
        vecIdx = i;
    }
}
rotvec = rotvec*cos(vecIdx*10) + (planevec.cross(rotvec))*sin(vecIdx*10);
rotvec = rotvec.normalized();

pose[3] = acos(planevec(0));
pose[4] = asin(rotvec(1)); //in radians
pose[5] = atan2(rotvec(0),rotvec(2));
}
float* Get_values(){
    return pose;
}
};

float Dist(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int index1, int index2){ //between points
float distx = cloud->points[index1].x - cloud->points[index2].x;
float disty = cloud->points[index1].y - cloud->points[index2].y;
float distz = cloud->points[index1].z - cloud->points[index2].z;

float len = sqrt(pow(distx, 2.0) + pow(disty, 2.0) + pow(distz, 2.0));
return len;
}

float Dist(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int index1){ //to origin
float len = sqrt(pow(cloud->points[index1].x, 2.0) + pow(cloud->points[index1].y, 2.0) + pow(cloud->points[index1].z, 2.0));
return len;
}

tuple<int, int> eighteen (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int start, int end){
float distx = cloud->points[start].x - cloud->points[end].x;
float disty = cloud->points[start].y - cloud->points[end].y;
float distz = cloud->points[start].z - cloud->points[end].z;
//Finding the length of the line
float len1 = sqrt(pow(distx, 2.0) + pow(disty, 2.0) + pow(distz, 2.0));

float difflen = 0.14/len1;

//Scaling the line to 0.18
len1 *= difflen;

//By modifying t we can find any point on the line
//where t = 0 corresponds to our starting point and t = 1 to our end point
//any value of t in between will be a point on the line between start and end
float t = -1 * difflen;
//Here we calculate the direction "vector"
float directionx = cloud->points[start].x - cloud->points[end].x;
float directiony = cloud->points[start].y - cloud->points[end].y;
float directionz = cloud->points[start].z - cloud->points[end].z;

//Pushing the point on the line 18cm from start to the pointcloud
//Notice that each point is calculated by adding the start point with the direction vector multiplied by t
pcl::PointXYZ pp;
pp.x = cloud->points[start].x + directionx * t; pp.y = cloud->points[start].y + directiony * t; pp.z = cloud->points[start].z + directionz * t;
cloud->push_back(pp);

cout << cloud->points.size() << endl;
//Here we find the distance from the start of the line to the point
int lastindex = cloud->points.size()-1;
float distpointx = cloud->points[start].x - cloud->points[lastindex].x;
float distpointy = cloud->points[start].y - cloud->points[lastindex].y;
float distpointz = cloud->points[start].z - cloud->points[lastindex].z;
float lenToPoint = sqrt(pow(distpointx, 2.0) + pow(distpointy, 2.0) + pow(distpointz, 2.0));
float lenToPoint1 = lenToPoint;
//Here we search for the smallest distance to the 18cm point from any point in the pointclooud
//The shortst distance to the point will then be the last i value -1 since the smallest distance to the point is the distance from the point to the itself
int p1 = 0, p2 = 0;
vector<int> temp;
for(int i = 0; i < cloud->points.size(); i++)
{
    if (i!= cloud->points.size()-1)
    {
        float itepointx = cloud->points[i].x - cloud->points[lastindex].x;
        float itepointy = cloud->points[i].y - cloud->points[lastindex].y;
        float itepointz = cloud->points[i].z - cloud->points[lastindex].z;

        if(lenToPoint > sqrt(pow(itepointx, 2.0) + pow(itepointy, 2.0) + pow(itepointz, 2.0)))
        {
            lenToPoint = sqrt(pow(itepointx, 2.0) + pow(itepointy, 2.0) + pow(itepointz, 2.0));
            //temp.push_back(i);
            p1 = i;///temp.rbegin()[1];
        }
    } 
}
for(int i = 0; i < cloud->points.size(); i++)
{
    if (abs(p1-i)>30 && i!= cloud->points.size()-1)//exclude immediate neighbors
    {
        float itepointx = cloud->points[i].x - cloud->points[p1].x;
        float itepointy = cloud->points[i].y - cloud->points[p1].y;
        float itepointz = cloud->points[i].z - cloud->points[p1].z;

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
    for (int i = 0; i < arraySize/2; i++) //sum first half
    {
        sum += myArray[i];
    }
    avg1 = sum/arraySize/2; //mean of first half

    sum = 0.0;
    for (int i = arraySize/2; i < arraySize; i++)
    {
        sum += myArray[i];
    }
    avg2 = sum/arraySize/2;

    if (avg1<avg2) //compare
    {
        return true;
    } else {
        return false;
    }
}

void callBack(const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::PCLPointCloud2::Ptr cloud_input(new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredz (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredx (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredy (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_out(new pcl::PointCloud<pcl::PointXYZ>);

    //Using the PointCloud2 data from the ROS topic
    pcl_conversions::toPCL(*input, *cloud_input);

    // Convert to the templated PointCloud
    pcl::fromPCLPointCloud2(*cloud_input, *cloud);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits(0.95, 1.1);
    pass.filter(*cloud_filteredz);
  //1st: (0.95, 1.1)
    pass.setInputCloud(cloud_filteredz);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits(-0.2, 0.05);
    pass.filter(*cloud_filteredx);
  //1st: (-0.2, 0.05)
    pass.setInputCloud(cloud_filteredx);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits(-0.1, 0.3);
    pass.filter(*cloud_filteredy);
  //1st: (-0.1, 0.3)

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud_filteredy);
  seg.segment (*inliers, *coefficients);
  std::cerr << "PointCloud after segmentation has: "
            << inliers->indices.size () << " inliers." << std::endl;

  // Project the model inliers
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setIndices (inliers);
  proj.setInputCloud (cloud_filteredy);
  proj.setModelCoefficients (coefficients);
  proj.filter (*plane_out);
  std::cerr << "PointCloud after projection has: "
            << plane_out->points.size () << " data points." << std::endl;

  // Create a Concave Hull representation of the projected inliers
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ConcaveHull<pcl::PointXYZ> chull;
  chull.setInputCloud (plane_out);
  chull.setAlpha (0.01);
  chull.reconstruct (*cloud_hull);

  copyPointCloud(*cloud_hull, *cloud);

srand (static_cast <unsigned> (time(0)));

///////////////////////////////////////////////////////////////////////////
// Try out different pairs, looking for the longest distance between them
float longest = 0.0; // longest line found
float length = 0.0;  // current length being evaluated
int legTip = 0;      // one end of the line
int legEnd = 0;      // other end of the line

for (int i = 0; i < cloud->points.size()/2; i++)
{
    for (int j = 0; j < cloud->points.size(); j++)
    {
        if (abs(i-j)>cloud->points.size()/5) //exclude immediate neighbors
        {
            length = Dist(cloud, i, j); //calculate distance
            if (length>longest)
            {
                longest = length; //Assign new longest length
                legTip = i; //Assign leg tip index
                legEnd = j; //Assign leg end index
            }
        }
    }
}
cout << "The longest line found goes between points [" << legTip << "," << legEnd
     << "] and has length: " << longest << endl;

/////////////////////////////////////////////////////////////////////////////////
// Placing the cut based on: thickness increase. Index increment
const int gran = 1; //granularity, how close clsoe are the lines across the leg. 1 for most legs
const int iterations = cloud->points.size()/(gran*2); //how many line will be made
float shortest[iterations] = {1.0}; //distances between pairs of oneSide[i] and otherSide[i]
int oneSide[iterations] = {};
int otherSide[iterations] = {}; //nearest point opposite from an oneSide

// populate oneSide with indices, starting from legTip
for (int i = 0; i < iterations; i++)
{
    if (legTip + i*gran > cloud->points.size()-1)
    {// in case we would exceed size of point cloud
        oneSide[i] = legTip + i*gran - cloud->points.size();
    }
    else
    {
        oneSide[i] = legTip + i*gran;
    }
}
// find otherSide[i] for each oneSide[i]
for (int i = 0; i < iterations; i++)
{   //for each point assigned to oneSide
    shortest[i] = 1.0; //high initial value, to ensure that search happens
    for (int j = 0; j < cloud->points.size(); j++)
    {//search for nearest opposite
        if (abs(j-oneSide[i])>30)//exclude immediate neighbors
        {
            length = Dist(cloud, oneSide[i], j); //calculate distance between points
            if (length<shortest[i]) //if dist is less than previously found
            {
                shortest[i] = length; //Assign new shortest dist
                otherSide[i] = j; //Assign index for that point to otherSide array
            }
        }
    }
}
/////////////////////////////////////////////////////////////////////////////////
// Make sure that legTip is the narrow end
if (startIsNarrower(shortest, iterations))
{
    cout << "legTip is in the narrow end, no switch" << endl;
} else {
    int temp = legTip; // switch them
    legTip = legEnd;
    legEnd = temp;
    cout << "legTip was in the wide end, has been switched" << endl;
}
/////////////////////////////////////////////////////////////////////////////////
// Discriminate lines based on angle // Calculate angles
Eigen::Vector3f alongVec, acrossVec;
alongVec << cloud->points[legEnd].x - cloud->points[legTip].x,
        cloud->points[legEnd].y - cloud->points[legTip].y,
        cloud->points[legEnd].z - cloud->points[legTip].z;
alongVec = alongVec.normalized(); // unit vector of the longest line along the leg

std::vector<int> valVec; //only valid lines 
uint valCount = 0; //How many lines are valid
float angle[iterations] = {}; //Angles of the lines
for (int i = 0; i < iterations; i++)
{
    acrossVec << cloud->points[oneSide[i]].x - cloud->points[otherSide[i]].x,
            cloud->points[oneSide[i]].y - cloud->points[otherSide[i]].y,
            cloud->points[oneSide[i]].z - cloud->points[otherSide[i]].z;
    acrossVec = acrossVec.normalized(); // unit vector of line across the leg
    
    float dotp = alongVec.transpose() * acrossVec;
    angle[i] = acos(dotp) * 180/3.14159265; //angle in deg

    if (abs(angle[i] - 90) < 35) //accept a diff of up to 35 deg
    {
        valVec.push_back(i+1); //positive indicates valid line
        valCount++; // count valid lines
    }
    else
    {
        valVec.push_back(0); // 0 indicates invalid line
    }
}
//Make new array with only valid lines
float valShort[valCount] = {}; //lengths of valid lines
uint relcount = 0;
for (int i = 0; i < iterations; i++)
{
    if (valVec[i]) //if this index's line is valid
    {
        valShort[relcount] = shortest[i];
        relcount++;
    }
}
///////////////////////////////////////////////////////////////////////////////
// Evaluate thickness // Find local minimum, starting from the middle
uint minIdx = valCount/2; //Where we start searching
bool minFound = false; //has the best local minimum been found?
uint minTries = 0; //times we have hit a local minimum
uint check = valCount/12; //Check a nr of steps, proportional to size of array
int dir = 1; //is positive to ensure that it can move, if it starts in a local minimum
while(!minFound)
{
    if (valShort[minIdx]>valShort[minIdx-1])
    {
        minIdx--;
        dir = -1;
        cout << "minus" << endl;
    }
    else if (valShort[minIdx]>valShort[minIdx+1])
    {
        minIdx++;
        dir = 1;
        cout << "plus" << endl;
    }
    else if (valShort[minIdx]<valShort[minIdx+1] && valShort[minIdx]<valShort[minIdx-1])
    {
        cout << "Possible min at: " << minIdx << endl;
        for (int i = 1; i <= check; i++)
        {
            if (valShort[minIdx + i*dir] < valShort[minIdx]) //Check some indices ahead for lower value
            {
                minIdx = minIdx + i*dir; //set the search index there
                i = check; //make sure the loop stops
            }
        }
        if (!dir)
        {
            minFound = true;
            cout << "Found min: [" << minIdx << "] with value: [" << valShort[minIdx] << "]" << endl;
        }
        if (minTries < 1) //if we haven't hit a local minimum before
        {
            dir = -dir; //swap direction
            minTries++;
        } else {
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
int finIdx = 0; //
for (int i = 0; i < iterations; i++)
{
    if (finIdx == minIdx - 1)
    {
        finIdx = i*gran;
        i = iterations;
    }
    if (valVec[i])
    {
        finIdx++;
    }
}
cout << "Min has original idx: [" << finIdx << "] with value: [" << shortest[finIdx] << "]" << endl;

////////////////////////////////////////////////////////////////////////////////
// Placing the cut based on: distance from tip, 0.18m
int point1, point2; //coordinates where the cut should be
tie(point1, point2) = eighteen(cloud, legTip, legEnd);
int lastindex = cloud->points.size()-1;


if (cloud->poins.size() <= 200)
{int interval_tolerance = 5;}
else {int interval_tolerance = int(cloud->points.size() / 40);}

/////////////////////////////////////////////////////////////////////////////////
// Combine the two estimates
if (abs(finIdx-point1)<cloud->points.size()/40 || abs(finIdx-point2)<cloud->points.size()/40)
{// if the two estimates are close
    point1 = otherSide[finIdx]; //Use the finIdx found by local minimum search
    point2 = oneSide[finIdx];
}

///////////////////////////////////////////////////////////////////////////////
// Output concluded end effector pose
Pose myPose;
myPose.calculatePose(cloud, point1, point2, legTip, legEnd);
float *pose1 = myPose.Get_values();

cout << pose1[0] << endl;
cout << pose1[1] << endl;
cout << pose1[2] << endl;
cout << pose1[3] << endl;
cout << pose1[4] << endl;
cout << pose1[5] << endl;


std_msgs::Float32MultiArray output;
output.data.clear();

for(int i = 0; i < 6; i++)
{
    output.data.push_back(pose1[i]);
}

pub.publish(output);

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
    if (valVec[j])
    {
        if (j == finIdx) //concluded index for thickness evaluation
        {
            viewer.addLine(cloud->points[oneSide[j]], cloud->points[otherSide[j]], 1, 1, 0, str); //concluded line
        }
        else
        {
            viewer.addLine(cloud->points[oneSide[j]], cloud->points[otherSide[j]], 1, 0, 0, str); //between points
        }
    }
}
viewer.addLine(cloud->points[legTip], cloud->points[legEnd], 0, 1, 0, "q"); //Show longest line
viewer.addLine(cloud->points[point1], cloud->points[point2], 1, 1, 1, "t"); //18cm line

while(!viewer.wasStopped ())
{
    viewer.spinOnce ();
}
}

int main (int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "my_pcl_tutorial");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/kinect2/sd/points", 1, callBack);

    // Create a ROS publisher in the gloabal space, for use in the callBack function, for the output model coefficients
    pub = nh.advertise<std_msgs::Float32MultiArray>("/robotPose", 1000);

    ros::Rate loop_rate(10);

    while (ros::ok){
      ros::spinOnce();
      loop_rate.sleep();
    }

    return 0;
}
