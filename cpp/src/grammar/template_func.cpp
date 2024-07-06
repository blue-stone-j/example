#include <iostream>

template <class T>
void output(T a)
{
  std::cout << a << std::endl;
}

int main()
{
  // 隐式实例化
  std::cout << "隐式实例化输出" << std::endl;
  output(1);   // 整型
  output(1.2); // 浮点型
  std::cout << "显式实例化输出" << std::endl;
  // 显示实例化
  output<int>(34);        // 整型
  output<double>(3.1415); // 浮点型
  return 0;
}