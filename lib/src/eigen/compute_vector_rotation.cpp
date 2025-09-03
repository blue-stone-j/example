#include <iostream>
#include <Eigen/Dense>
#include <vector>

Eigen::VectorXd fitPoly(const Eigen::VectorXd &x, const Eigen::VectorXd &y, int order)
{
  Eigen::MatrixXd A(x.size(), order + 1);
  for (int i = 0; i <= order; ++i)
  {
    A.col(i) = x.array().pow(i);
  }

  // Solve for the coefficients using QR decomposition
  Eigen::VectorXd coefficients = A.householderQr().solve(y);
  return coefficients;
}

/**
 * @brief Compute the optimal rotation matrix using the Kabsch algorithm.
 * @param normals1 First set of normal vectors (each a 3D Eigen::Vector3d)
 * @param normals2 Second set of normal vectors (each a 3D Eigen::Vector3d)
 * @return 3×3 rotation matrix (Eigen::Matrix3d)
 */
Eigen::Matrix3d computeOptimalRotation(
    const std::vector<Eigen::Vector3d> &normals1,
    const std::vector<Eigen::Vector3d> &normals2)
{
  assert(normals1.size() == normals2.size());
  size_t N = normals1.size();

  Eigen::MatrixXd A(3, N);
  Eigen::MatrixXd B(3, N);

  for (size_t i = 0; i < N; ++i)
  {
    A.col(i) = normals1[i].normalized();
    B.col(i) = normals2[i].normalized();
  }

  Eigen::Matrix3d H = A * B.transpose();
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();

  Eigen::Matrix3d R = V * U.transpose();

  // Ensure a proper rotation (determinant = +1)
  if (R.determinant() < 0)
  {
    V.col(2) *= -1;
    R = V * U.transpose();
  }

  return R;
}


int main()
{
  // Eigen::VectorXd x(5), y(5);
  // x << 1, 2, 3, 4, 5;
  // y << 1, 4, 9, 16, 25;

  // Eigen::VectorXd coefficients = fitPoly(x, y, 9);

  // std::cout << "Fitted coefficients:\n"
  //           << coefficients << std::endl;

  std::vector<Eigen::Vector3d> normals1 = {
      {1, 0.5, 0.4},
      {0.999, 0.53, 0.36},
      {0.998, 0.48, 0.43}};

  std::vector<Eigen::Vector3d> normals2 = {
      {0.3, 0.4, 0.5},
      {0.31, 0.39, 0.51},
      {0.29, 0.38, 0.48}};

  Eigen::Matrix3d R = computeOptimalRotation(normals1, normals2);
  std::cout << "Optimal Rotation Matrix:\n"
            << R << std::endl;

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