#include <iostream>

int a = 10; // 全局变量a

int main()
{
  float a = 9.0;                 // 局部变量a
  a       = a - 1;               // 局部
  std::cout << a << std::endl;   // 输出局部变量值
  ::a = ::a - 1;                 // 全局变量减1
  std::cout << ::a << std::endl; // 输出全局变量值

  return 0;
}