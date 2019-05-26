#include <pcl/range_image/range_image.h>
#include <math.h>
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
#include <pcl/filters/passthrough.h>

using namespace std;

void pointPickingEventOccurred(const pcl::visualization::PointPickingEvent& event, void* viewer_void)
{
	std::cout << "[INOF] Point picking event occurred." << std::endl;

	float x, y, z;
	if (event.getPointIndex() == -1)
	{
		return;
	}
	event.getPoint(x, y, z);
	std::cout << "[INOF] Point coordinate ( " << x << ", " << y << ", " << z << ")" << std::endl;
}

void checkSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredz(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredx(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredy(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud_input);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.95, 1.1);
	pass.filter(*cloud_filteredz);
	//1st: (0.95, 1.1)
	pass.setInputCloud(cloud_filteredz);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-0.2, 0.05);
	pass.filter(*cloud_filteredx);
	//1st: (-0.2, 0.05)
	pass.setInputCloud(cloud_filteredx);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-0.1, 0.3);
	pass.filter(*cloud_filteredy);

	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	viewer.setBackgroundColor(0, 0, 0);
	viewer.addPointCloud<pcl::PointXYZ>(cloud_filteredy, "sample cloud");

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
}

int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("0002_cloud.pcd", *cloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}
	std::cout << "Loaded " << endl;

	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	//viewer->setBackgroundColor(0, 0, 0);
	viewer.addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	//viewer->addCoordinateSystem(1.0);
	//viewer->initCameraParameters();

	viewer.registerPointPickingCallback(pointPickingEventOccurred, (void*)&viewer);
	viewer.spin();

	//checkSegmentation(cloud);

	return 0;
}
