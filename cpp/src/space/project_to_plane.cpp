/*
project a point to a plane and return the projected point.
*/

#include <iostream>
#include <Eigen/Dense>

Eigen::Vector3d projectPointToPlane(const Eigen::Vector3d &point, const Eigen::Vector4d &plane)
{
  Eigen::Vector3d normal(plane[0], plane[1], plane[2]); // Normal vector (A, B, C)
  double D = plane[3];

  // Compute the signed distance from point to the plane
  double d = (normal.dot(point) + D) / normal.squaredNorm();

  // Compute the projected point
  Eigen::Vector3d projected_point = point - d * normal;

  return projected_point;
}

int main()
{
  Eigen::Vector3d point(3.0, 4.0, 5.0);       // Example point
  Eigen::Vector4d plane(1.0, 2.0, 3.0, -6.0); // Plane equation: x + 2y + 3z - 6 = 0

  Eigen::Vector3d projected_point = projectPointToPlane(point, plane);

  std::cout << "Projected Point: " << projected_point.transpose() << std::endl;

  return 0;
}