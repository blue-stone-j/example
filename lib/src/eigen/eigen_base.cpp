#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

int main()
{
  Eigen::Matrix4d mat4d = Eigen::Matrix4d::Identity();

  auto block = mat4d.topLeftCorner(3, 3); // block has type Eigen::Block<Eigen::Matrix4d,3,3>

  Eigen::Matrix3d sub = mat4d.topLeftCorner(3, 3); // copy as a standalone Matrix3d

  Eigen::Quaterniond q1(mat4d.topLeftCorner<3, 3>()); // Fixed-size corner

  // Eigen::Quaterniond q2(mat4d.topLeftCorner(3, 3).eval()); // Runtime-sized corner

  return 0;
}

/*
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

cv::Mat rotation_vec; // input rotation vector (3x1)
cv::Rodrigues(rotation_vec, R); // convert to rotation matrix

// Convert cv::Mat to Eigen::Matrix3d
Eigen::Matrix3d eigen_R;
cv::cv2eigen(R, eigen_R);  // requires #include <opencv2/eigen.hpp>

Eigen::Quaterniond q(eigen_R); // rotation quaternion
*/