#include <iostream>
#include <cstdio>
#include <cstring>
#include <fstream>

using namespace std;
const int max_n = 105;
struct Matrix
{
  int n, m;
  int v[max_n][max_n];
  Matrix(int n, int m) :
    n(n), m(m) {}
  void init()
  { // 初始化矩阵
    memset(v, 0, sizeof v);
  }
  Matrix operator*(const Matrix B) const
  {
    Matrix C(n, B.m); // 用来存放答案
    C.init();
    for (int i = 0; i < n; i++)
      for (int j = 0; j < B.m; j++)
        for (int k = 0; k < m; k++)
          C.v[i][j] += v[i][k] * B.v[k][j];
    return C;
  }
  void print()
  { // 输出该矩阵，用来测试
    for (int i = 0; i < n; i++)
    {
      for (int j = 0; j < m; j++)
      {
        cout << v[i][j] << " ";
      }
      cout << endl;
    }
  }
};

int main()
{
  ifstream file("Matrix.txt");
  if (!file.is_open())
  {
    cout << "wait" << endl;
    [[maybe_unused]] int result = system("pause");
  }
  else
  {
    cout << "right" << endl;
  }
  int n1 = 3, m1 = 3, n2 = 3, m2 = 3;

  Matrix A(n1, m1);
  Matrix B(n2, m2);
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      file >> A.v[i][j];
      B.v[i][j] = A.v[i][j];
    }
  }
  A.print();
  B.print();
  cout << n1 << endl;

  Matrix C = A * B;
  C.print();
  file.close();
  return 0;
}
