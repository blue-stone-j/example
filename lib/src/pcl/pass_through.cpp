/*
filter cloud points by specified field value range using PassThrough filter
*/

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

int main()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("x");   // filter based on x coordinate
  pass.setFilterLimits(0.0, 1.0); // keep points where 0 <= x <= 1
  pass.filter(*cloud_filtered);

  return 0;
}
