#include <pcl/range_image/range_image.h>
#include <math.h>
#include <ctime>
#include <cstdlib>
#include <Eigen/Dense>
//#include <pcl/io/eigen.h>
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
      MatrixXd d;
      d = MatrixXd::Random(5,5);
      Vector
      cout << d << endl;


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
      int idx1 = 0;
      int idx2 = 0;

      for (int i = 0; i < cloud->points.size() / 2; i++)
      {
            float xdist = cloud->points[i].x - cloud->points[cloud->points.size() / 2 + i].x;
            float ydist = cloud->points[i].y - cloud->points[cloud->points.size() / 2 + i].y;
            float zdist = cloud->points[i].z - cloud->points[cloud->points.size() / 2 + i].z;
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
                  float xdist = cloud->points[i].x - cloud->points[cloud->points.size() / 2 + i + 10].x;
                  float ydist = cloud->points[i].y - cloud->points[cloud->points.size() / 2 + i + 10].y;
                  float zdist = cloud->points[i].z - cloud->points[cloud->points.size() / 2 + i + 10].z;
                  len = sqrt(pow(xdist, 2.0) + pow(ydist, 2.0) + pow(zdist, 2.0));
            }
            else {
                  float xdist = cloud->points[i].x - cloud->points[cloud->points.size() / 2 + i - 10].x;
                  float ydist = cloud->points[i].y - cloud->points[cloud->points.size() / 2 + i - 10].y;
                  float zdist = cloud->points[i].z - cloud->points[cloud->points.size() / 2 + i - 10].z;
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

      //////////////////////////////////////////////////////////////////////////////////////
      // Create the segmentation object 
      /*
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      seg.setModelType (pcl::SACMODEL_LINE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setDistanceThreshold (0.03);
      seg.setInputCloud (cloud); 
      seg.segment (*inliers, *line_coefficients);
      /*const auto line_x = line_coefficients->values[0] = cloud->points[idx1].x;
      const auto line_y = line_coefficients->values[1] = cloud->points[idx1].y;
      const auto line_z = line_coefficients->values[2] = cloud->points[idx1].z;
      
      float direc_x = line_coefficients->values[0] = cloud->points[idx2].x - cloud->points[idx1].x;
      float direc_y = line_coefficients->values[1] = cloud->points[idx2].y - cloud->points[idx1].y;
      float direc_z = line_coefficients->values[2] = cloud->points[idx2].z - cloud->points[idx1].z;
      */
      //cout << "line x = " << line_x << " line y = " << line_y << " line z = " << line_z << endl;
      //cout << "direction x = " << direc_x << " direction y = " << direc_y << " direction z = " << direc_z << endl;
      

      /////////////////////////////////////////////////////////////////////////////////
      // Placing the cut based on: thickness increase. Stochastic skeletonization
      float segmentLength = longest/20;
      int iterations = 5;
      int next = 1;
      int idx3;
      int idx4;
      float coordArr[20][3];


      //Pick a point at the end of the leg, where the skeletonization starts
      coordArr[0][0] = {cloud->points[idx1].x};
      coordArr[0][1] = {cloud->points[idx1].y};
      coordArr[0][2] = {cloud->points[idx1].z};
      cout << "skeletonization starting point: " << endl;
      cout << coordArr[0][0] << endl;
      cout << coordArr[0][1] << endl;
      cout << coordArr[0][2] << endl;


      //Make points roughly in the direction of the line
      float tempArr[iterations][3] = {};
      float LO = 0.5; //for random generator
      float HI = 2.0;
      float ra;
      float shortest1 = 1;
      float shortest2 = 1;


      for (int i = 0; i < iterations; i++)
      {
            ra = LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI-LO)));
            tempArr[i][0] = coordArr[0][0] + segmentLength*(cloud->points[idx2].x - coordArr[0][0])*ra;
            
            ra = LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI-LO)));
            tempArr[i][1] = coordArr[0][1] + segmentLength*(cloud->points[idx2].y - coordArr[0][1])*ra;
            
            ra = LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI-LO)));
            tempArr[i][2] = coordArr[0][2] + segmentLength*(cloud->points[idx2].z - coordArr[0][2])*ra;
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
                  float xdist = tempArr[i][0] - cloud->points[point].x;
                  float ydist = tempArr[i][1] - cloud->points[point].y;
                  float zdist = tempArr[i][2] - cloud->points[point].z;
                  len = sqrt(pow(xdist, 2.0) + pow(ydist, 2.0) + pow(zdist, 2.0));

                  if (len<shortest1)
                  {
                        shortest1 = len;
                        idx3 = point;
                  }
            }
            //When nearest is found, find opposite nearest
            
      }
      //Reapeat
      /*
      cout << ra << endl;
      for (int i = 0; i < 10; ++i)
      {
            ra = LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI-LO)));
            cout << ra << endl;
      }*/
      //Until the width crosses some threshold


      ////////////////////////////////////////////////////////////////////////////////
      // Placing the cut based on: distance from tip, 180mm





      ////////////////////////////////////////////////////////////////////////////////
      pcl::visualization::PCLVisualizer viewer("PCL Viewer");
      viewer.setBackgroundColor (0, 0, 0);
      viewer.addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");

      for(int i=0; i < cloud->points.size()-1; i++){
      stringstream ss;
      ss << i;
      string str = ss.str();
      viewer.addLine(cloud->points[i], cloud->points[i+1], 1, 1, 1, str); //Make lines between points
      }

      viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud,normals);
      viewer.addLine(cloud->points[idx1], cloud->points[idx2], 0, 1, 0, "q"); //Show longest line

      //viewer.addLine(normals->points[0], normals->points[normals->points.size()-1], 1,0 ,0, "p");
      while(!viewer.wasStopped ())
      {
      viewer.spinOnce ();
      }
return 0;
}
