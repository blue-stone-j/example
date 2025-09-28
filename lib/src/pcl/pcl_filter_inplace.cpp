#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>
#include <algorithm>

template <class PointT>
void filterByRadiusInPlace(pcl::PointCloud<PointT> &cloud,
                           const Eigen::Vector3f &center, float radius)
{
  const float r2 = radius * radius;
  const float cx = center.x(), cy = center.y(), cz = center.z();

  std::size_t w = 0; // write cursor
  for (std::size_t r = 0; r < cloud.points.size(); ++r)
  {
    const auto &p  = cloud.points[r];
    const float dx = p.x - cx, dy = p.y - cy, dz = p.z - cz;
    if (dx * dx + dy * dy + dz * dz <= r2)
    {
      cloud.points[w++] = std::move(cloud.points[r]);
    }
  }
  cloud.points.resize(w);
  cloud.width    = static_cast<std::uint32_t>(w);
  cloud.height   = 1;    // became unorganized
  cloud.is_dense = true; // set accordingly if you kept NaNs
}


template <class PointT>
void filterByRadiusEraseRemove(pcl::PointCloud<PointT> &cloud,
                               const Eigen::Vector3f &c, float radius)
{
  const float r2 = radius * radius;
  const float cx = c.x(), cy = c.y(), cz = c.z();

  auto it = std::remove_if(cloud.points.begin(), cloud.points.end(),
                           [&](const PointT &p) {
                             const float dx = p.x - cx, dy = p.y - cy, dz = p.z - cz;
                             return dx * dx + dy * dy + dz * dz > r2; // remove distant
                           });
  cloud.points.erase(it, cloud.points.end());
  cloud.width    = static_cast<std::uint32_t>(cloud.points.size());
  cloud.height   = 1;
  cloud.is_dense = true;
}

int main(int argc, char **argv)
{
}