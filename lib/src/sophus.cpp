#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/so3.hpp>

int main()
{
  // Create SO3 from rotation vector (example: 90° around Z-axis)
  Eigen::Vector3d rot_vec(0, 0, M_PI / 2);
  Sophus::SO3d so3(rot_vec);

  // Convert to Eigen quaternion
  Eigen::Quaterniond q = so3.unit_quaternion();

  std::cout << "Quaternion (x, y, z, w): "
            << q.x() << ", "
            << q.y() << ", "
            << q.z() << ", "
            << q.w() << std::endl;

  return 0;
}
