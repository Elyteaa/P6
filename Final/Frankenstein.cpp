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
#include <pcl_ros/transforms.h>

//ROS
#include "geometry_msgs/Pose.h"
#include <sensor_msgs/PointCloud2.h>

//Might not need these
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include <pcl/range_image/range_image.h> //
#include <math.h> //
#include <vector> //
#include <pcl/io/io.h> //

#include <pcl/io/pcd_io.h>
//#include <pcl/features/integral_image_normal.h> //
#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/filters/voxel_grid.h> //
//#include <pcl/io/ply_io.h> //
//#include <pcl/features/normal_3d.h> //
//#include <iostream>
//#include <Eigen/Geometry>
//#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

ros::Publisher pub;
ros::Publisher cloud_pubb;

/*pcl::fromPCLPointCloud(*cloud_input, *cloud);
pcl::PassThrough<pcl::PointZYX> pass;
pass.setInputCloud(cloud);
pass.setFilterFieldName ("z");
pass.setFilterLimits(0.95, 1.1);
pass.filter(*cloud_filteredz);
*/

float Dist(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int index1, int index2){ //Between points
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

class Pose {
private:
    float pose[7];
public:
void calculatePose(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int start1, int end1, int start2, int end2){
//Make the point indicating position
pose[0] = cloud->points[end1].x + (cloud->points[start1].x - cloud->points[end1].x)/2;
pose[1] = cloud->points[end1].y + (cloud->points[start1].y - cloud->points[end1].y)/2;
pose[2] = cloud->points[end1].z + (cloud->points[start1].z - cloud->points[end1].z)/2;

//Define vector between sides of leg
Eigen::Vector3f across, along, rotvec, planevec, robvec;
across << cloud->points[end1].x - cloud->points[start1].x,
          cloud->points[end1].y - cloud->points[start1].y,
          cloud->points[end1].z - cloud->points[start1].z;
across = across.normalized();

//Define vector that is the longest line
along << cloud->points[end2].x - cloud->points[start2].x,
         cloud->points[end2].y - cloud->points[start2].y,
         cloud->points[end2].z - cloud->points[start2].z;
along = along.normalized();

//Local minimum isn't accurate by itself. Shift position 1cm towards shank
float posShift = 0.01; //Shift 1 cm
pose[0] = pose[0] + along(0)*posShift;
pose[1] = pose[1] + along(1)*posShift;
pose[2] = pose[2] + along(2)*posShift;

rotvec = across.cross(along); //Vector to be rotated around the plane vector
rotvec = rotvec.normalized();
planevec = across.cross(rotvec); //Vector to define cutting plane
planevec = planevec.normalized();
robvec << pose[0], pose[1], pose[2]; //Unit vector from the leg to the robot
robvec = robvec.normalized();

float angle[36] = {}; //Contains found angles
float smallAngle = 360; //Smallest found angle. high initial value
float angleInc = 10*(3.14159265/180); //Angle increment in radians, 10 deg
int vecIdx; //Which vector fits best
float dotp; //Dotproduct

for (int i = 0; i < 36; i++)
{ //Test which vector fits best
    rotvec = rotvec*cos(angleInc) + (planevec.cross(rotvec))*sin(angleInc) + planevec*(planevec.dot(rotvec))*(1-cos(angleInc)); //Rodrigue's formula
    dotp = rotvec.dot(robvec); // Dot product with robot vector
    angle[i] = acos(dotp) * 180/3.14159265; //Angle in deg
    cout << "Angle[" << i << "]: " << angle[i] << endl;
    if (angle[i]<smallAngle)
    {
        smallAngle = angle[i]; //Assign new smallest angle
        vecIdx = i;            //Assign idx of best fitting vector
    }
}
cout << "Smallest angle: [" << vecIdx << "]: " << smallAngle << endl;

rotvec = rotvec*cos(vecIdx*angleInc) + (planevec.cross(rotvec))*sin(vecIdx*angleInc) + planevec*(planevec.dot(rotvec))*(1-cos(angleInc)); //Rodrigue's formula
rotvec = rotvec.normalized();
// Convert orientation into quaternions 
Eigen:: Quaternionf quaternionForRotation;
// RPY In radians



float roll = acos(planevec(0)); //roll
float pitch = asin(rotvec(1)); //pitch
float yaw = atan2(rotvec(0),rotvec(2)); //yaw
//Converting orientation to quaternions
cout << "R, P, Y: " << roll << ", " << pitch << ", " << yaw << endl;



quaternionForRotation =   AngleAxisf(roll, Vector3f::UnitX())
    * AngleAxisf(pitch, Vector3f::UnitY())
    * AngleAxisf(yaw, Vector3f::UnitZ());
//cout << "Quaternion" << endl << q.coeffs() << endl;


quaternionForRotation.FromTwoVectors(robvec, rotvec);
//cout << "Quaternion" << endl << q.coeffs() << endl;

/*
Eigen::Vector3f vectorFromEndEffector, crossFromVectors;
vectorFromEndEffector << 0.54, -1.32, 0.04; //what's this?
vectorFromEndEffector = vectorFromEndEffector.normalized();
crossFromVectors = planevec.cross(vectorFromEndEffector);

float lengthofPlanevec = sqrt(pow(planevec.x(),2) + pow(planevec.y(),2) + pow(planevec.z(),2));
float lengthofvectorFromEnfeffector = sqrt(pow(vectorFromEndEffector.x(),2) + pow(vectorFromEndEffector.y(),2) + pow(vectorFromEndEffector.z(),2));

Eigen::Quaternionf quaternionForRotation;

quaternionForRotation.x() = crossFromVectors.x();
quaternionForRotation.y() = crossFromVectors.y();
quaternionForRotation.z() = crossFromVectors.z();
quaternionForRotation.w() = planevec.dot(vectorFromEndEffector) + sqrt(lengthofPlanevec * lengthofvectorFromEnfeffector);

cout << "Real part = " << quaternionForRotation.w() << " Vector part = " << endl << quaternionForRotation.vec() << endl;

quaternionForRotation = quaternionForRotation.normalized();
*/
pose[3] = quaternionForRotation.x();
pose[4] = quaternionForRotation.y();
pose[5] = quaternionForRotation.z();
pose[6] = quaternionForRotation.w();
}
float* get_Pose(){
return pose;
}
};

tuple<int, int> setDistFunction (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int start, int end){
float legLength = Dist(cloud, start, end); //Finding the length of the line
float setDist = 0.15; //Set dist from start to desired new point
float lengthRatio = setDist/legLength; //Ratio of set length to leg length

//Here we calculate the direction "vector"
float xDir = cloud->points[end].x - cloud->points[start].x;
float yDir = cloud->points[end].y - cloud->points[start].y;
float zDir = cloud->points[end].z - cloud->points[start].z;

pcl::PointXYZ newPoint;             //Create empty point newPoint
newPoint.x = cloud->points[start].x + xDir*lengthRatio;
newPoint.y = cloud->points[start].y + yDir*lengthRatio; //Calculate its position
newPoint.z = cloud->points[start].z + zDir*lengthRatio;
cloud->push_back(newPoint);         //Add new point to point cloud

//Search for the smallest dist to the new point from any point, except itself
int newPointIndex = cloud->points.size()-1; //Index of the new point
int p1 = 0, p2 = 0;                 //Indices of best line across leg
float tempDist, tempShortest = 1.0; //Temp variables for finding shortest dist
for(int i = 0; i < cloud->points.size(); i++)
{
    if (i!= newPointIndex) //Exclude the new point
    {
        tempDist = Dist(cloud, i, newPointIndex); //Dist to a point
        if(tempShortest > tempDist) //If current point is closer than any found before
        {
            tempShortest = tempDist;//Assign new shortest dist
            p1 = i;                 //Assign index for closest point
        }
    } 
}
tempShortest = 1.0; //Reset to high value
for(int i = 0; i < cloud->points.size(); i++) //Search for closest point on the other side
{
    if (abs(p1-i)>30 && i!= newPointIndex)//Exclude close neighbors and new point
    {
        tempDist = Dist(cloud, i, p1); //Dist to a point
        if(tempShortest > tempDist)    //If current point is closer than any found before
        {
            tempShortest = tempDist;//Assign new shortest dist
            p2 = i;                 //Assign index for closest point
        }
    }
}
cout << "Set dist estimate: [" << p1 << ", " << p2 << "]" << endl;
return make_tuple(p1, p2);
}

bool startIsTip(float myArray[], int arraySize, std::vector<int> &myVec, int vecSize){
//Checks if the legTip index is assigned to the tip end of the pcd
float sum = 0.0, temp = 0.0, avg1, avg2, var1, var2;
for (int i = 0; i < arraySize/2; i++) //Sum first half of array
{
    sum += myArray[i];
}
avg1 = sum/arraySize/2; //Mean of 1st half
for (int i = 0; i < arraySize/2; i++)
{
    temp += (myArray[i] - avg1) * (myArray[i] - avg1);
}
var1 = temp/(arraySize/2); //Variance of 1st half

sum = temp = 0.0; //reset sum and temp
for (int i = arraySize/2; i < arraySize; i++) //Sum 2nd half of array
{
    sum += myArray[i];
}
avg2 = sum/arraySize/2; //Mean of 2nd half
for (int i = 0; i < arraySize/2; i++)
{
    temp += (myArray[i] - avg2) * (myArray[i] - avg2);
}
var2 = temp/(arraySize/2); //Variance of 2nd half

// Count nr of invalid lines in each half of the leg
float inv1 = 0, inv2 = 0;
for (int i = 0; i < vecSize/2; i++)
{
    if (!myVec[i]) //myVec[i]=0 indicates line [i] is invalid
    {
        inv1++;
    }
}
for (int i = vecSize/2; i < vecSize; i++)
{
    if (!myVec[i])
    {
        inv2++;
    }
}
cout << "avg1: " << avg1 << endl;
cout << "avg2: " << avg2 << endl;
cout << "var1: " << var1 << endl;
cout << "var2: " << var2 << endl;
cout << "inv1: " << inv1 << endl;
cout << "inv2: " << inv2 << endl;
//Evaluate which half the tip belongs to
int firstHalf = 0, secondHalf = 0; //Count "points"
float certainty = abs(avg1-avg2)/(avg1+avg2) + abs(var1-var2)/(var1+var2) + abs(inv1-inv2)/(inv1+inv2);
float cert1 = abs(avg1-avg2)/(avg1+avg2);
float cert2 = abs(var1-var2)/(var1+var2);
float cert3 = abs(inv1-inv2)/(inv1+inv2);

if (avg1<avg2) //Compare averages
{//Smaller average distance across favours a given end of the leg
    cout << "Smaller mean -> firstHalf" << endl;
    firstHalf++;
} else {
    cout << "Smaller mean -> secondHalf" << endl;
    secondHalf++;
}
if (var1<var2) //Compare variances
{//Greater variance favours a given end of the leg
    cout << "Greater variance -> secondHalf" << endl;
    secondHalf++;
} else {
    cout << "Greater variance -> firstHalf" << endl;
    firstHalf++;
}
if (inv1<inv2) //Compare nr of invalid lines
{//Less invalid lines favours a given end of the leg
    cout << "Fewer invalid lines -> firstHalf" << endl;
    firstHalf++;
} else {
    cout << "Fewer invalid lines -> secondHalf" << endl;
    secondHalf++;
}
 
cout << "Score: firstHalf half: [" << firstHalf << "]" << endl;
cout << "Score: secondHalf half: [" << secondHalf << "]" << endl;
cout << "cert1: " << cert1 << endl;
cout << "cert2: " << cert2 << endl;
cout << "cert3: " << cert3 << endl;
cout << "certainty: " << certainty << endl;
if (firstHalf>secondHalf) //If more indicators favour the first half
{
    return true; //first half is selected
} else {
    return false; //second half is selected
}
}

void callBack(const sensor_msgs::PointCloud2ConstPtr& input)
{
srand (static_cast <unsigned> (time(0)));
// load point cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pub (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr plane_out (new pcl::PointCloud<pcl::PointXYZ>);


pcl::io::loadPCDFile ("0000_cloud.pcd", *cloud_input); //prev: hulltest.pdc
cout << "Point cloud size: " << cloud_input->points.size() << endl;

   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredz (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredx (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredy (new pcl::PointCloud<pcl::PointXYZ>);

//pcl_conversions::toPCL(*input, *cloud_input);
    // Convert to the templated PointCloud
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_input);
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


///////////////////////////////////////////////////////////////////////////
// Try out different pairs, looking for the longest distance between them
float longest = 0.0; // Longest line found
float length = 0.0;  // Current length being evaluated
int legTip = 0;      // One end of the longest line, supposed to be leg tip
int legShank = 0;    // Other end of the longest line, supposed to be shoulder

for (int i = 0; i < cloud->points.size()/2; i++)
{
    for (int j = 0; j < cloud->points.size(); j++)
    {
        if (abs(i-j)>cloud->points.size()/5) //Exclude immediate neighbors
        {
            length = Dist(cloud, i, j); //Calculate distance
            if (length>longest)
            {
                longest = length;   //Assign new longest length
                legTip = i;         //Assign leg tip index
                legShank = j;       //Assign leg end index
            }
        }
    }
}
cout << "The longest line found goes between points [" << legTip << ", " << legShank
     << "] and has length: " << longest << endl;
/////////////////////////////////////////////////////////////////////////////////
// Placing the cut based on: thickness increase. Index increment
const int iterations = cloud->points.size()/2;//Nr of lines across the leg
float shortest[iterations] = {};    //Distances between pairs of oneSide[i] and otherSide[i]
int oneSide[iterations] = {};       //Index of a point on one side of the leg
int otherSide[iterations] = {};     //Index of nearest point opposite from oneSide[i]
// Populate oneSide with indices, starting from legTip
for (int i = 0; i < iterations; i++)
{
    if (legTip + i > cloud->points.size()-1)
    {// In case we would exceed size of point cloud
        oneSide[i] = legTip + i - cloud->points.size();
    } else {
        oneSide[i] = legTip + i;
    }
}
// Find otherSide[i] for each oneSide[i]
for (int i = 0; i < iterations; i++)
{   //For each point assigned to oneSide
    shortest[i] = 1.0; //High initial value, to ensure that search happens
    for (int j = 0; j < cloud->points.size(); j++)
    {//Search for nearest opposite
        if (abs(j-oneSide[i])>30)//Exclude immediate neighbors
        {
            length = Dist(cloud, oneSide[i], j); //Calculate distance between points
            if (length<shortest[i]) //If dist is less than previously found
            {
                shortest[i] = length; //Assign new shortest dist
                otherSide[i] = j; //Assign index for that point to otherSide array
            }
        }
    }
}
/////////////////////////////////////////////////////////////////////////////////
// Discriminate between lines based on angle // Calculate angles
Eigen::Vector3f alongVec, acrossVec;
alongVec << cloud->points[legShank].x - cloud->points[legTip].x,
            cloud->points[legShank].y - cloud->points[legTip].y,
            cloud->points[legShank].z - cloud->points[legTip].z;
alongVec = alongVec.normalized(); //Unit vector of the longest line along the leg

std::vector<int> valVec; //Vector that shows which lines are valid
int valCount = 0; //How many lines are valid
float angle[iterations] = {}; //Angles of the lines
for (int i = 0; i < iterations; i++)
{
    acrossVec << cloud->points[oneSide[i]].x - cloud->points[otherSide[i]].x,
                 cloud->points[oneSide[i]].y - cloud->points[otherSide[i]].y,
                 cloud->points[oneSide[i]].z - cloud->points[otherSide[i]].z;
    acrossVec = acrossVec.normalized(); //Unit vector of line across the leg
    
    float dotp = alongVec.transpose() * acrossVec;
    angle[i] = acos(dotp) * 180/3.14159265; //Angle in deg

    if (abs(angle[i] - 90) < 25) //Accept a diff of up to 35 deg
    {
        valVec.push_back(i+1); //Positive indicates valid line
        valCount++; //Count valid lines
    }
    else
    {
        valVec.push_back(0); // 0 indicates invalid line
    }
}
//Make new array with only valid lines
float valShort[valCount] = {}; //Lengths of valid lines
int relCount = 0;
for (int i = 0; i < iterations; i++)
{
    if (valVec[i]) //If this index's line is valid
    {
        valShort[relCount] = shortest[i];
        relCount++;
    }
}
/////////////////////////////////////////////////////////////////////////////////
// Make sure that legTip is the narrow end
if (startIsTip(valShort, valCount, valVec, iterations))
{
    cout << "legTip is in the narrow end, no swap" << endl;
} else {
    int temp = legTip; // Switch them
    legTip = legShank;
    legShank = temp;
    cout << "legTip was in the wide end, has been swapped" << endl;
}
///////////////////////////////////////////////////////////////////////////////
// Evaluate thickness // Find local minimum
int minIdx = valCount/2;  //Where we start searching, in the middle
bool minFound = false;    //Has the best local minimum been found?
uint minTries = 0;        //Times we have hit a local minimum
uint check = valCount/12; //Check a nr of steps, proportional to size of array
int dir = -1;             //Direction. Is positive to ensure mobility, if start at local minimum
while(!minFound)
{
    if (minIdx == 0 || minIdx == valCount)
    {
        cout << "At the edge of the array!" << endl;
    }
    if (valShort[minIdx]>valShort[minIdx-1] && minIdx > 0)
    {
        minIdx--; //Decrement: search at lower indices
        dir = -1; //Set direction negative
        cout << "Minimum search: -" << endl;
    }
    else if (valShort[minIdx]>valShort[minIdx+1] && minIdx < valCount)
    {
        minIdx++; //Increment: search at higher indices
        dir = 1;  //Set direction pos
        cout << "Minimum search: +" << endl;
    }
    else if (valShort[minIdx]<valShort[minIdx+1] && valShort[minIdx]<valShort[minIdx-1])
    {
        cout << "Local min at: " << minIdx << endl;
        for (int i = 1; i <= check; i++)
        {
            if (valShort[minIdx+i*dir]<valShort[minIdx] && (valCount-minIdx)>check && minIdx>check) //Check some indices ahead for lower value
            {
                minIdx = minIdx + i*dir; //Set the search index there
                i = check; //Make sure the loop stops
                cout << "Skipped " << i*dir << " to " << minIdx << endl;
            }
        }
        if (!dir)
        {
            minFound = true;
            cout << "Found min: [" << minIdx << "] with value: [" << valShort[minIdx] << "]" << endl;
        }
        if (minTries < 1) //If we haven't hit a local minimum before
        {
            dir = -dir; //Swap direction
            minTries++;
        } else {
            dir = 0;
        }
    } else {
        cout << "Error in finding minimum!" << endl;
        break;
    }
}
// Go from minIdx back to real idx of local minimum
int finIdx = 0; //Final index, real index of concluded point
for (int i = 0; i < iterations; i++)
{
    if (finIdx == minIdx - 1)
    {
        finIdx = i;
        i = iterations;
    }
    if (valVec[i])
    {
        finIdx++;
    }
}
cout << "Min has original idx: [" << oneSide[finIdx] << ", " << otherSide[finIdx] << "] with value: [" << shortest[finIdx] << "]" << endl;
////////////////////////////////////////////////////////////////////////////////
// Placing the cut based on: distance from tip, 0.18m
int point1, point2; //Coordinates where the cut should be
tie(point1, point2) = setDistFunction(cloud, legTip, legShank);

/////////////////////////////////////////////////////////////////////////////////
// Combine the two estimates
if (abs(finIdx-point1)<=cloud->points.size()/40 || abs(finIdx-point2)<=cloud->points.size()/40)
{// If the two estimates are close
    point1 = oneSide[finIdx]; //Use the finIdx found by local minimum search
    point2 = otherSide[finIdx];
}
///////////////////////////////////////////////////////////////////////////////
// Output concluded end effector pose
Pose myPose;
myPose.calculatePose(cloud, point1, point2, legTip, legShank);
float *pose = myPose.get_Pose();

cout << "End effector pose:" << endl;
cout << "x: " << pose[0] << endl;
cout << "y: " << pose[1] << endl;
cout << "z: " << pose[2] << endl;
cout << "Quaternion part:" << endl;
cout << "w: " << pose[3] << endl;
cout << "x: " << pose[4] << endl;
cout << "y: " << pose[5] << endl;
cout << "z: " << pose[6] << endl;



geometry_msgs::Pose output;
pub.publish(output);

sensor_msgs::PointCloud2 output_cloud;
pcl::toROSMsg(*cloud_input, output_cloud);


cloud_pubb.publish(output_cloud);

output.position.x = pose[0];
output.position.y = pose[1];
output.position.z = pose[2];
output.orientation.w = pose[3];
output.orientation.x = pose[4];
output.orientation.y = pose[5];
output.orientation.z = pose[6];

}

int main (int argc, char** argv) {
    while (ros::ok)
    {

// Initialize ROS
    ros::init(argc, argv, "my_pcl_tutorial");
    ros::NodeHandle nh;

ros::Subscriber sub = nh.subscribe("/kinect2/sd/points", 1, callBack);
    // Create a ROS publisher for the output model coefficients
    pub = nh.advertise<geometry_msgs::Pose>("/robotPose", 1000);
cloud_pubb = nh.advertise<sensor_msgs::PointCloud2> ("/captureThis", 1);






///////////////////////////////////////////////////////////////////////////////
// Viewer //
/*
pcl::visualization::PCLVisualizer viewer("PCL Viewer");
viewer.setBackgroundColor (0, 0, 0);
viewer.addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
viewer.addCoordinateSystem(0.05, pose[0], pose[1], pose[2]);

for(int j = 0; j < iterations; j++) //Idx-wise lines across the leg
{
    stringstream ss;
    ss << j;
    string str = ss.str();
    if (valVec[j])
    {
        if (j == finIdx) //Concluded index for thickness evaluation
        {
            viewer.addLine(cloud->points[oneSide[j]], cloud->points[otherSide[j]], 1, 1, 0, str); //
        }
        else
        {
            viewer.addLine(cloud->points[oneSide[j]], cloud->points[otherSide[j]], 1, 0, 0, str); //between points
        }
    }
}
viewer.addLine(cloud->points[legTip], cloud->points[legEnd], 0, 1, 0, "q"); //Show longest line
viewer.addLine(cloud->points[point1], cloud->points[point2], 1, 1, 1, "t"); //18cm line
*/
/*
while(!viewer.wasStopped ())
{
    viewer.spinOnce ();
}*/

ros::Rate loop_rate(10);


      ros::spinOnce();
      loop_rate.sleep();
}

return 0;
}
