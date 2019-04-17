#include <pcl/range_image/range_image.h>
#include <math.h>
#include <vector>
#include <ctime>
#include <cstdlib>
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
using Eigen::MatrixXd;


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
      float longest = 0.0;
      float len = 0.0;
      float xdist = 0.0;
      float ydist = 0.0;
      float zdist = 0.0;
      int idx1 = 0;
      int idx2 = 0;

      for (int i = 0; i < cloud->points.size() / 2; i++)
      {
            xdist = cloud->points[i].x - cloud->points[cloud->points.size() / 2 + i].x;
            ydist = cloud->points[i].y - cloud->points[cloud->points.size() / 2 + i].y;
            zdist = cloud->points[i].z - cloud->points[cloud->points.size() / 2 + i].z;
            len = sqrt(pow(xdist, 2.0) + pow(ydist, 2.0) + pow(zdist, 2.0));

            if (len>longest)
            {
                  longest = len;
                  idx1 = i;
                  idx2 = cloud->points.size() / 2 + i;
            }
      }
      // Try with shifted upper index
      for (int i = 0; i < cloud->points.size() / 2; ++i)
      {
            if ((i % 2) && (cloud->points.size() / 2 + i + 10 <= cloud->points.size())
                  && (cloud->points.size() / 2 + i + 10 >= 0))
            {
                  xdist = cloud->points[i].x - cloud->points[cloud->points.size() / 2 + i + 10].x;
                  ydist = cloud->points[i].y - cloud->points[cloud->points.size() / 2 + i + 10].y;
                  zdist = cloud->points[i].z - cloud->points[cloud->points.size() / 2 + i + 10].z;
                  len = sqrt(pow(xdist, 2.0) + pow(ydist, 2.0) + pow(zdist, 2.0));
            }
            else {
                  xdist = cloud->points[i].x - cloud->points[cloud->points.size() / 2 + i - 10].x;
                  ydist = cloud->points[i].y - cloud->points[cloud->points.size() / 2 + i - 10].y;
                  zdist = cloud->points[i].z - cloud->points[cloud->points.size() / 2 + i - 10].z;
                  len = sqrt(pow(xdist, 2.0) + pow(ydist, 2.0) + pow(zdist, 2.0));
            }

            if (len>longest)
            {
                  longest = len;
                  idx1 = i;
                  if (i % 2)
                  {
                        idx2 = cloud->points.size() / 2 + i + 10;
                  }
                  else
                  {
                        idx2 = cloud->points.size() / 2 + i - 10;
                  }
            }
      }

      cout << "The longest line goes between points [" << idx1 << "," << idx2
             << "] and has length: " << longest << endl;

      /////////////////////////////////////////////////////////////////////////////////
      // Placing the cut based on: thickness increase. Stochastic skeletonization
      int iterations = 5; //nr of randomized coordinates pr segment
      int seg = 20; //nr of segments
      float segLen = longest/seg;
      int idx3; //nearest point to a randomized coord
      int idx4; //nearest point opposite from idx3
      float coordArr[20][3]; //stores

      //Pick a point at the end of the leg, where the skeletonization starts
      coordArr[0][0] = {cloud->points[idx1].x};
      coordArr[0][1] = {cloud->points[idx1].y};
      coordArr[0][2] = {cloud->points[idx1].z};
      cout << "skeletonization starting point: " << endl;
      cout << coordArr[0][0] << endl;
      cout << coordArr[0][1] << endl;
      cout << coordArr[0][2] << endl;

      //Make "points" roughly in the direction of the line
      float tempArr[iterations][3] = {}; //stores temporary randomized coordinates
      float LO = 0.5; //lower bound for ra
      float HI = 2.0; //upper bound for ra
      float ra; // random number
      float shortest[iterations]; //dist between opposite points
      float middiff[iterations]; //difference in distance to left and right nearest points
      float dist3; //dist to idx 3 from to point
      float dist4;

      vector<int> searchSet;


      for (int i = 0; i < iterations; i++) //generate slightly randomized coordinates
      {
            ra = LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI-LO)));
            tempArr[i][0] = coordArr[0][0] + segLen*(cloud->points[idx2].x - coordArr[0][0])*ra;
            
            ra = LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI-LO)));
            tempArr[i][1] = coordArr[0][1] + segLen*(cloud->points[idx2].y - coordArr[0][1])*ra;
            
            ra = LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI-LO)));
            tempArr[i][2] = coordArr[0][2] + segLen*(cloud->points[idx2].z - coordArr[0][2])*ra;
      }

      cout << "proposed next point" << endl;
      cout << tempArr[iterations-1][0] << endl;
      cout << tempArr[iterations-1][1] << endl;
      cout << tempArr[iterations-1][2] << endl;

      //Pick the one which is most in the center, check width
      for (int i = 0; i < iterations; i++) //for each random point
      {
            for (int point = 0; point < cloud->points.size(); point++) //search for nearest
            {
                  xdist = tempArr[i][0] - cloud->points[point].x;
                  ydist = tempArr[i][1] - cloud->points[point].y;
                  zdist = tempArr[i][2] - cloud->points[point].z;
                  len = sqrt(pow(xdist, 2.0) + pow(ydist, 2.0) + pow(zdist, 2.0));

                  if (len<shortest[i])
                  {
                        shortest[i] = len;
                        idx3 = point;
                  }
            }

            if (abs(idx3 - idx1)<20) //if nearest point is close to the tip
            {
                  if (idx3>idx1) //if nearest point is to the right
                  {
                        if ((idx1-idx3)<0) //if the tip is close to idx(0)
                        {
                              idx4 = cloud->points.size() - (idx3 - idx1);
                        }
                        else
                        {
                              idx4 = idx1 - idx3;
                        }
                  }
                  else //if nearest is to the left
                  {
                        if ((idx1 + idx3) >= cloud->points.size()) //if the tip is close to idx(0)
                        {
                              idx4 = (idx1 - idx3) - (cloud->points.size() - idx3);
                        }
                        else
                        {
                              idx4 = idx1 + idx3;
                        }
                  }
                  //Calculate distance between the two opposite nearest points
                        cout << "hellllllooooo" << endl;
                  xdist = cloud->points[idx4].x - cloud->points[idx3].x;
                  ydist = cloud->points[idx4].y - cloud->points[idx3].y;
                  zdist = cloud->points[idx4].z - cloud->points[idx3].z;
                  shortest[i] = sqrt(pow(xdist, 2.0) + pow(ydist, 2.0) + pow(zdist, 2.0));
            }
            else //if nearest point is not close to the tip
            {
                  shortest[i] = 1.0;
                  for (int point = 0; point < cloud->points.size(); point++)
                  {//search for nearest opposite
                        if (abs(point-idx3)<20)
                        {
                              xdist = tempArr[i][0] - cloud->points[point].x;
                              ydist = tempArr[i][1] - cloud->points[point].y;
                              zdist = tempArr[i][2] - cloud->points[point].z;
                              len = sqrt(pow(xdist, 2.0) + pow(ydist, 2.0) + pow(zdist, 2.0));

                              if (len<shortest[i])
                              {
                                    shortest[i] = len;
                                    idx4 = point;
                              }
                        }
                  }
            }
            //calculate how close it is to the center
            xdist = tempArr[i][0] - cloud->points[idx3].x;
            ydist = tempArr[i][1] - cloud->points[idx3].y;
            zdist = tempArr[i][2] - cloud->points[idx3].z;
            dist3 = sqrt(pow(xdist, 2.0) + pow(ydist, 2.0) + pow(zdist, 2.0));

            xdist = tempArr[i][0] - cloud->points[idx4].x;
            ydist = tempArr[i][1] - cloud->points[idx4].y;
            zdist = tempArr[i][2] - cloud->points[idx4].z;
            dist4 = sqrt(pow(xdist, 2.0) + pow(ydist, 2.0) + pow(zdist, 2.0));

            middiff[i] = abs(dist3-dist4);
      }
      //pick the point that is most in the middle

      float leastdiff = 1.0;
      for (int i = 0; i < iterations; i++)
      {
            if (middiff[i]<leastdiff)
            {
                  leastdiff = middiff[i];
            }
            cout << i << endl;
      }



      searchSet = {1,2};
      cout << searchSet[0] << " " << searchSet[1] << endl;
      //Reapeat

      //Until the width crosses some threshold


      ////////////////////////////////////////////////////////////////////////////////
      // Placing the cut based on: distance from tip, 180mm
      /*
      coordArr[1][0] = cloud->points[idx1].x + 0.18*(cloud->points[idx2].x - cloud.points[idx1].x);
      coordArr[1][2] = cloud->points[idx1].y + 0.18*(cloud->points[idx2].y - cloud.points[idx1].y);
      coordArr[1][3] = cloud->points[idx1].z + 0.18*(cloud->points[idx2].z - cloud.points[idx1].z);
      */





      ////////////////////////////////////////////////////////////////////////////////
      pcl::visualization::PCLVisualizer viewer("PCL Viewer");
      viewer.setBackgroundColor (0, 0, 0);
      viewer.addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");

      for(int i=0; i < cloud->points.size()-1; i++)
      {
            stringstream ss;
            ss << i;
            string str = ss.str();
            viewer.addLine(cloud->points[i], cloud->points[i+1], 1, 1, 1, str); //Make lines between points
      }

      viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud,normals);
      viewer.addLine(cloud->points[idx1], cloud->points[idx2], 0, 1, 0, "q"); //Show longest line
      viewer.addLine(cloud->points[20], cloud->points[160], 1, 0, 0, "a"); //line across leg
      viewer.addLine(cloud->points[21], cloud->points[159], 1, 0, 0, "b"); //line across leg
      viewer.addLine(cloud->points[22], cloud->points[158], 1, 0, 0, "c"); //line across leg
      viewer.addLine(cloud->points[idx3], cloud->points[idx4], 1, 0, 0, "d"); //line across leg


      while(!viewer.wasStopped ())
      {
      viewer.spinOnce ();
      }
return 0;
}
