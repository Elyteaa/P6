#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/io.h>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <iostream>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <vector>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/filters/normal_refinement.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include <pcl/common/geometry.h>

#include <pcl/filters/project_inliers.h>
#include <pcl/filters/passthrough.h>

using namespace std;

unsigned int text_id = 0;

void mouseEventOccurred(const pcl::visualization::MouseEvent &event, void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *>(viewer_void);

  if (event.getButton() == pcl::visualization::MouseEvent::LeftButton &&
      event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease)
    std::cout << "Left mouse button released at position (" << event.getX() << ", " << event.getY() << ")" << std::endl;
}


int main(int argc, char** argv)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr plane_out(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_out1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredz (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredx (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredy (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::io::loadPCDFile <pcl::PointXYZ>("maximus.pcd", *cloud);

	pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits(0.5, 1.7);
    pass.filter(*cloud_filteredz);
  
    pass.setInputCloud(cloud_filteredz);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits(-0.2, 0.1);
    pass.filter(*cloud_filteredx);

    pass.setInputCloud(cloud_filteredx);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits(-0.245, 0.211);
    pass.filter(*cloud_filteredy);


  // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  // pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // // Create the segmentation object
  // pcl::SACSegmentation<pcl::PointXYZ> seg;
  // // Optional
  // seg.setOptimizeCoefficients (true);
  // // Mandatory
  // seg.setModelType (pcl::SACMODEL_PLANE);
  // seg.setMethodType (pcl::SAC_RANSAC);
  // seg.setDistanceThreshold (0.1);

  // seg.setInputCloud (cloud_filteredy);
  // seg.segment (*inliers, *coefficients);
  // std::cerr << "PointCloud after segmentation has: "
  //           << inliers->indices.size () << " inliers." << std::endl;

  // // Project the model inliers-
  // pcl::ProjectInliers<pcl::PointXYZ> proj;
  // proj.setModelType (pcl::SACMODEL_PLANE);
  // proj.setIndices (inliers);
  // proj.setInputCloud (cloud_filteredy);
  // proj.setModelCoefficients (coefficients);
  // proj.filter (*plane_out);
  // std::cerr << "PointCloud after projection has: "
  //           << plane_out->points.size () << " data points." << std::endl;

  // // Create a Concave Hull representation of the projected inliers
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::ConcaveHull<pcl::PointXYZ> chull;
  // chull.setInputCloud (plane_out);
  // chull.setAlpha (0.01);
  // chull.reconstruct (*cloud_hull);

  //pcl::io::savePCDFileASCII("finallegv3.pcd", *cloud_filteredy);


	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	viewer.setBackgroundColor(0.0, 0.0, 0.0);
	//viewer.addPointCloud<pcl::PointXYZ> (plane_out1, "sample cloud");
	viewer.addPointCloud<pcl::PointXYZ>(cloud_filteredx, "sample plane");
	viewer.addCoordinateSystem(0.1);
  viewer.registerMouseCallback(mouseEventOccurred, (void*)&viewer);
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}

	return 0;
}
