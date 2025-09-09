#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_search.h>
#include <iostream>
#include <vector>

int main()
{
  // Load point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>("input.pcd", *cloud) == -1)
  {
    PCL_ERROR("Couldn't read input file\n");
    return -1;
  }

  // Create an octree with resolution 5cm
  float resolution = 0.05f;
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
  octree.setInputCloud(cloud);
  octree.addPointsFromInputCloud();

  // Choose a search point (e.g., the first point in the cloud)
  pcl::PointXYZ searchPoint = cloud->points[0];

  // --- K Nearest Neighbor Search ---
  int K = 5;
  std::vector<int> k_indices;
  std::vector<float> k_sqr_distances;

  if (octree.nearestKSearch(searchPoint, K, k_indices, k_sqr_distances) > 0)
  {
    std::cout << "K nearest neighbors:\n";
    for (size_t i = 0; i < k_indices.size(); ++i)
    {
      std::cout << "    " << cloud->points[k_indices[i]].x << " "
                << cloud->points[k_indices[i]].y << " "
                << cloud->points[k_indices[i]].z
                << " (distance: " << k_sqr_distances[i] << ")\n";
    }
  }

  // --- Radius Search ---
  float radius = 0.1f; // 10 cm search radius
  std::vector<int> radius_indices;
  std::vector<float> radius_sqr_distances;

  if (octree.radiusSearch(searchPoint, radius, radius_indices, radius_sqr_distances) > 0)
  {
    std::cout << "\nNeighbors within radius " << radius << ":\n";
    for (size_t i = 0; i < radius_indices.size(); ++i)
    {
      std::cout << "    " << cloud->points[radius_indices[i]].x << " "
                << cloud->points[radius_indices[i]].y << " "
                << cloud->points[radius_indices[i]].z
                << " (distance: " << radius_sqr_distances[i] << ")\n";
    }
  }

  return 0;
}
