#include <iostream>
#include <string>
#include <cstring>

template <class T>
void output(T a)
{
  std::cout << "参数类型为：" << typeid(a).name() << std::endl;

  if (strstr(typeid(a).name(), "char") != NULL) // 有子串char
  {
    if (strlen(typeid(a).name()) == 4) // 字符
    {}
    else // 字符串
    {
      std::cout << "字符串：" << a << std::endl;
    }
  }
}
int main()
{
  std::string str = "13"; // 字符串
  output(str);            // 输出

  int var1 = 1; // 整型
  output(var1); // 输出

  double var2 = 2.4545; // 浮点型
  output(var2);         // 输出

  char ch[] = "1234"; // C风格字符串
  output(ch);

  output('d'); // 字符

  output("689"); // 字符串常量

  return 0;
}