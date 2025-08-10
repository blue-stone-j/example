#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/octree.h>

int main()
{
  // Load point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>("input.pcd", *cloud) == -1)
  {
    PCL_ERROR("Couldn't read input file\n");
    return -1;
  }

  // Create an octree-based search object
  pcl::search::Octree<pcl::PointXYZ>::Ptr octree(new pcl::search::Octree<pcl::PointXYZ>(0.05)); // 5cm resolution
  octree->setInputCloud(cloud);
  //   octree->addPointsFromInputCloud();

  // Compute normals
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  ne.setSearchMethod(octree);
  ne.setKSearch(10); // Use the 10 nearest neighbors
  ne.compute(*normals);

  // Save the output
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
  pcl::io::savePCDFileASCII("output_with_normals.pcd", *cloud_with_normals);

  return 0;
}
