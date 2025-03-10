
/*
receive a normal vector and return a rotation matrix that aligns the normal vector to the Z-axis.
*/

#include <Eigen/Dense>
#include <iostream>

Eigen::Matrix3d getRotationMatrixToXYPlane(const Eigen::Vector3d &normal)
{
  Eigen::Vector3d target(0, 0, 1);                                      // Desired normal to XOY plane
  Eigen::Vector3d axis = normal.cross(target);                          // Rotation axis
  double angle         = std::acos(normal.dot(target) / normal.norm()); // Rotation angle

  if (axis.norm() < 1e-6)
  {
    return Eigen::Matrix3d::Identity(); // Already aligned
  }

  axis.normalize(); // Normalize rotation axis

  Eigen::AngleAxisd rotation(angle, axis); // Rodrigues' rotation
  return rotation.toRotationMatrix();
}

int main()
{
  Eigen::Vector3d normal(1, 1, 1); // Example normal
  Eigen::Matrix3d R = getRotationMatrixToXYPlane(normal);

  std::cout << "Rotation Matrix:\n"
            << R << std::endl;

  // Verify transformation
  Eigen::Vector3d transformed_normal = R * normal;
  std::cout << "Transformed Normal: " << transformed_normal.transpose() << std::endl;

  return 0;
}