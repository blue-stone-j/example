/**
 * @file main.cpp
 * @brief Normal-based region growing segmentation using PCL
 * @details
 *   Usage:
 *     ./normal_region_growing input_cloud.pcd [input_normals.pcd]
 *
 *   If input_normals.pcd is omitted, the program will estimate normals.
 *
 *   Outputs:
 *     - clusters/cluster_00000.pcd, cluster_00001.pcd, ...
 *
 * @author bs
 * @LastEditTime 2025-09-25
 */

#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <cmath>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/features/normal_3d_omp.h>

namespace fs = std::filesystem;

int main(int argc, char **argv)
{
  if (argc < 2 || argc > 3)
  {
    std::cout << "Usage: " << argv[0] << " input_cloud.pcd [input_normals.pcd]\n";
    return 1;
  }

  const std::string cloud_path   = argv[1];
  const bool have_normals_file   = (argc == 3);
  const std::string normals_path = have_normals_file ? argv[2] : "";

  using PointT       = pcl::PointXYZ;
  using CloudT       = pcl::PointCloud<PointT>;
  using NormalT      = pcl::Normal;
  using NormalCloudT = pcl::PointCloud<NormalT>;

  CloudT::Ptr cloud(new CloudT);
  if (pcl::io::loadPCDFile<PointT>(cloud_path, *cloud) != 0)
  {
    std::cerr << "Error: failed to load point cloud: " << cloud_path << "\n";
    return 2;
  }
  if (cloud->empty())
  {
    std::cerr << "Error: input cloud is empty.\n";
    return 3;
  }
  std::cout << "Loaded cloud: " << cloud_path << " with " << cloud->size() << " points.\n";

  NormalCloudT::Ptr normals(new NormalCloudT);

  if (have_normals_file)
  {
    if (pcl::io::loadPCDFile<NormalT>(normals_path, *normals) != 0)
    {
      std::cerr << "Error: failed to load normals: " << normals_path << "\n";
      return 4;
    }
    if (normals->size() != cloud->size())
    {
      std::cerr << "Error: normals size (" << normals->size()
                << ") does not match cloud size (" << cloud->size() << ").\n";
      return 5;
    }
    std::cout << "Loaded normals: " << normals_path << "\n";
  }
  else
  {
    std::cout << "Normals not provided. Estimating with NormalEstimationOMP...\n";
    pcl::NormalEstimationOMP<PointT, NormalT> ne;
    ne.setInputCloud(cloud);

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    ne.setSearchMethod(tree);

    // Choose either K or radius. Here we use K nearest neighbors for robustness.
    const int k_neighbors = 30;
    ne.setKSearch(k_neighbors);

    ne.compute(*normals);
    std::cout << "Estimated normals for " << normals->size() << " points.\n";
  }

  // Build the search tree for segmentation
  pcl::search::Search<PointT>::Ptr search_tree(new pcl::search::KdTree<PointT>);

  // Region Growing segmentation (normal-based smoothness + curvature)
  pcl::RegionGrowing<PointT, NormalT> reg;
  reg.setInputCloud(cloud);
  reg.setInputNormals(normals);
  reg.setSearchMethod(search_tree);

  // Tunable parameters:
  reg.setMinClusterSize(50);
  reg.setMaxClusterSize(static_cast<int>(cloud->size()));
  reg.setNumberOfNeighbours(30);

  // Angle threshold between neighboring normals (radians). 3 degrees default.
  const double smoothness_deg = 3.0;
  reg.setSmoothnessThreshold(smoothness_deg * M_PI / 180.0);

  // Maximum allowed point curvature inside a cluster
  reg.setCurvatureThreshold(1.0);

  std::vector<pcl::PointIndices> clusters;
  reg.extract(clusters);

  std::cout << "Found " << clusters.size() << " clusters.\n";

  // Create output directory
  const fs::path out_dir("clusters");
  std::error_code ec;
  fs::create_directories(out_dir, ec);
  if (ec)
  {
    std::cerr << "Warning: cannot create output directory 'clusters': " << ec.message() << "\n";
  }

  // Save clusters
  std::size_t total_points_in_clusters = 0;
  for (std::size_t i = 0; i < clusters.size(); ++i)
  {
    const pcl::PointIndices &indices = clusters[i];

    CloudT::Ptr cluster_cloud(new CloudT);
    cluster_cloud->reserve(indices.indices.size());
    for (int idx : indices.indices)
    {
      cluster_cloud->push_back((*cloud)[static_cast<std::size_t>(idx)]);
    }
    total_points_in_clusters += cluster_cloud->size();

    char namebuf[64];
    std::snprintf(namebuf, sizeof(namebuf), "cluster_%05zu.pcd", i);
    const fs::path filepath = out_dir / namebuf;

    if (pcl::io::savePCDFileBinary(filepath.string(), *cluster_cloud) != 0)
    {
      std::cerr << "Warning: failed to save " << filepath << "\n";
    }
    else
    {
      std::cout << "Saved: " << filepath << " (" << cluster_cloud->size() << " pts)\n";
    }
  }

  std::cout << "Clustered points total: " << total_points_in_clusters
            << " / " << cloud->size() << " ("
            << (100.0 * static_cast<double>(total_points_in_clusters) / static_cast<double>(cloud->size()))
            << "%)\n";

  // Optional: colorized result (uncomment to write a colored cloud of all clusters)
  // {
  //     pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored(new pcl::PointCloud<pcl::PointXYZRGB>);
  //     colored->reserve(cloud->size());
  //     std::vector<uint8_t> palette_r, palette_g, palette_b;
  //     // Simple repeating palette:
  //     for (int k = 0; k < 64; ++k) {
  //         palette_r.push_back(static_cast<uint8_t>((37 * k) % 255));
  //         palette_g.push_back(static_cast<uint8_t>((91 * k) % 255));
  //         palette_b.push_back(static_cast<uint8_t>((173 * k) % 255));
  //     }
  //
  //     std::vector<int> label(cloud->size(), -1);
  //     for (std::size_t i = 0; i < clusters.size(); ++i)
  //         for (int idx : clusters[i].indices) label[static_cast<std::size_t>(idx)] = static_cast<int>(i);
  //
  //     for (std::size_t i = 0; i < cloud->size(); ++i)
  //     {
  //         pcl::PointXYZRGB p;
  //         p.x = (*cloud)[i].x; p.y = (*cloud)[i].y; p.z = (*cloud)[i].z;
  //         int lbl = label[i];
  //         if (lbl >= 0) {
  //             const int c = lbl % 64;
  //             p.r = palette_r[c]; p.g = palette_g[c]; p.b = palette_b[c];
  //         } else {
  //             p.r = 128; p.g = 128; p.b = 128;
  //         }
  //         colored->push_back(p);
  //     }
  //     pcl::io::savePCDFileBinary("clusters_colorized.pcd", *colored);
  //     std::cout << "Saved colorized clusters: clusters_colorized.pcd\n";
  // }

  std::cout << "Done.\n";
  return 0;
}
