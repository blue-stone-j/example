#include <iostream>
#include <iomanip>
#include <string>

void print_int(int x, int y)
{
  std::cout << x << ' ' << y << std::endl;             // 按十进制输出
  std::cout << std::oct << x << ' ' << y << std::endl; // 按八进制输出
  std::cout << std::hex << x << ' ' << y << std::endl; // 按十六进制输出
}
void print_float(float a, float b)
{
  std::cout << a << ' ' << b << std::endl; // 无格式输出
  std::cout.setf(std::ios::showpos);       // 强制在正数前加+号
  std::cout << a << ' ' << b << std::endl;
  std::cout.unsetf(std::ios::showpos); // 取消正数前加+号
  std::cout.setf(std::ios::showpoint); // 强制显示小数点后的无效0
  std::cout << a << ' ' << b << std::endl;
  std::cout.unsetf(std::ios::showpoint); // 取消显示小数点后的无效0
  std::cout.setf(std::ios::scientific);  // 科学记数法
  std::cout << a << ' ' << b << std::endl;
  std::cout.unsetf(std::ios::scientific); // 取消科学记数法
  std::cout.setf(std::ios::fixed);        // 按点输出显示
  std::cout << a << ' ' << b << std::endl;
  std::cout.unsetf(std::ios::fixed); // 取消按点输出显示
  std::cout.precision(18);           // 精度为18，正常为6
  std::cout << a << ' ' << b << std::endl;
  std::cout.precision(6); // 精度恢复为6
}
void print_string(std::string a, std::string b, std::string *c, int length)
{
  std::cout << a << std::endl;
  std::cout << " ";
  std::cout << std::setiosflags(std::ios::left) << std::setw(10); // 设置宽度为10，left对齐
  std::cout << b << std::endl;
  std::cout << std::resetiosflags(std::ios::left); // 取消对齐方式
  for (int i = 0; i < length; i++)
  {
    std::cout << "  ";
    std::cout << std::setw(7) << std::setiosflags(std::ios::left); // 设置宽度为10，left对齐
    std::cout << c[i];
    std::cout << std::resetiosflags(std::ios::left);                       // 取消对齐方式
    std::cout << std::setfill('.') << std::setw(30) << i + 1 << std::endl; // 宽度为30，填充为'.'输出
  }
}
int main()
{
  int xx = 100, yy = 200;
  print_int(xx, yy); // 按进制输出

  float f1 = 0.000000001, f2 = -0.6;
  print_float(f1, f2); // 浮点数

  std::string str_caption = "第6章";
  std::string str1        = "实例087  格式打印（设计函数）";
  std::string str_sub[3]  = {"实例描述", "实现过程", "代码解析"};
  print_string(str_caption, str1, str_sub, 3); // 字符串输出

  return 0;
}