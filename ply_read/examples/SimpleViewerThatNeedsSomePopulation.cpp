 #include <pcl/visualization/cloud_viewer.h>
 //...
 int main()
 {
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
   //... populate cloud
   pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
   viewer.showCloud (cloud);
   while (!viewer.wasStopped ())
   {
   }
 }
