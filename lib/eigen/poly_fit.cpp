#include <iostream>
#include <Eigen/Dense>

Eigen::VectorXd polyfit(const Eigen::VectorXd &x, const Eigen::VectorXd &y, int order)
{
  Eigen::MatrixXd A(x.size(), order + 1);
  for (int i = 0; i <= order; ++i)
  {
    A.col(i) = x.array().pow(i);
  }

  // Solve for the coefficients using QR decomposition
  Eigen::VectorXd coeffs = A.householderQr().solve(y);
  return coeffs;
}

int main()
{
  Eigen::VectorXd x(5), y(5);
  x << 1, 2, 3, 4, 5;
  y << 1, 4, 9, 16, 25;

  Eigen::VectorXd coeffs = polyfit(x, y, 9);

  std::cout << "Fitted coefficients:\n"
            << coeffs << std::endl;
  return 0;
}
