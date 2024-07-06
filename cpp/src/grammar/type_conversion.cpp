#include <iostream>
#include <string>

void output(void *p, char ch)
{
  switch (ch)
  {
    case 'a': {
      int *ap = (int *)p; // 强制转换
      std::cout << *ap << " ";
    }
    break;
    case 'b': {
      char *chp = (char *)p; // 强制转换
      std::cout << *chp << " ";
    }
    break;
    case 'c': {
      double *dp = (double *)p; // 强制转换
      std::cout << *dp << " ";
    }
    break;
    case 'd': {
      std::string *strp = (std::string *)p; // 强制转换
      std::cout << *strp << " ";
    }
    break;
  }
  std::cout << std::endl;
}
int main()
{
  int a           = 4;       // int
  char ch         = 'a';     // ch
  double d        = 0.2145;  // double
  std::string str = "Hello"; // string
  output(&a, 'a');           // 输出整型
  output(&ch, 'b');          // char型
  output(&d, 'c');           // double型
  output(&str, 'd');         // 字符串

  return 0;
}