#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <iostream>

using namespace std;
int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), 
                                      cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), 
                                      cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDReader reader;

  reader.read ("goodie.pcd", *cloud);
  cout << cloud->isOrganized() << endl;
  // Build a filter to remove spurious NaNs
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 1.1);
  pass.filter (*cloud_filtered);
  std::cerr << "PointCloud after filtering has: "
            << cloud_filtered->points.size () << " data points." << std::endl;

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

  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);
  std::cerr << "PointCloud after segmentation has: "
            << inliers->indices.size () << " inliers." << std::endl;

  // Project the model inliers
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setIndices (inliers);
  proj.setInputCloud (cloud_filtered);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);
  std::cerr << "PointCloud after projection has: "
            << cloud_projected->points.size () << " data points." << std::endl;

  // Create a concave Hull representation of the projected inliers
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ConcaveHull<pcl::PointXYZ> chull;
  chull.setInputCloud(cloud_projected);
  chull.setAlpha(0.01);
  //chull.setDimension(2);
  chull.reconstruct(*cloud_hull);

  std::cerr << "concave hull has: " << cloud_hull->points.size ()
            << " data points." << std::endl;

  pcl::PCDWriter writer;
  writer.write ("concaveboi.pcd", *cloud_hull, false);

float x_min = -0.24;
float x_max = -0.035;
float y_min = -0.025;
float y_max = 0.169;

  for(int i; i < cloud_hull->points.size(); i ++)
  {
    
     if(cloud_hull->points[i].x >= x_min && cloud_hull->points[i].x <= x_max && cloud_hull->points[i].y >= y_min && cloud_hull->points[i].y <= y_max)
     {

      //cout << cloud_hull->points[i].x << "," << cloud_hull->points[i].y << endl;
      cout << cloud_hull->points[i].y << endl;

     }
    //cout << cloud_hull->points[i].x << "," << cloud_hull->points[i].y << endl;
     //cout << cloud_hull->points[i].x << endl;
  }

  return (0);
}
