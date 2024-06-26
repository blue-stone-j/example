#include <iostream>
#include <string>
#include <cmath>

#define NUM 4          // 4个数
#define RESULT 24      // 结果24
#define PRECISION 1E-6 // 精度

bool Cal(int n, double *number, std::string *equation)
{
  if (n == 1) // 计算结束
  {
    if (fabs(number[0] - RESULT) < PRECISION) // 小于精度
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  for (int i = 0; i < n - 1; i++) // 没结束，继续循环
  {
    for (int j = i + 1; j < n; j++)
    {
      double a, b;
      std::string expa, expb;
      a = number[i];
      b = number[j];
      // 挪动后面有效数字
      number[j]   = number[n - 1];
      expa        = equation[i];
      expb        = equation[j];
      equation[j] = equation[n - 1];
      // a+b
      equation[i] = '(' + expa + '+' + expb + ')';
      number[i]   = a + b;
      if (Cal(n - 1, number, equation))
      {
        return true;
      }
      // a-b
      equation[i] = '(' + expa + '-' + expb + ')';
      number[i]   = a - b;
      if (Cal(n - 1, number, equation))
      {
        return true;
      }
      // b-a
      equation[i] = '(' + expb + '-' + expa + ')';
      number[i]   = b - a;
      if (Cal(n - 1, number, equation))
      {
        return true;
      }
      //(a*b)
      equation[i] = '(' + expa + '*' + expb + ')';
      number[i]   = a * b;
      if (Cal(n - 1, number, equation))
      {
        return true;
      }
      //(a/b)
      if (b != 0) // 除数不为0
      {
        equation[i] = '(' + expa + '/' + expb + ')';
        number[i]   = a / b;
        if (Cal(n - 1, number, equation))
        {
          return true;
        }
      }
      //(b/a)
      if (a != 0) // 除数不为0
      {
        equation[i] = '(' + expb + '/' + expa + ')';
        number[i]   = b / a;
        if (Cal(n - 1, number, equation))
        {
          return true;
        }
      }
      // 恢复
      number[i]   = a;
      number[j]   = b;
      equation[i] = expa;
      equation[j] = expb;
    }
  }
  return false;
}
int main()
{
  double a[NUM];       // 4个数
  std::string eq[NUM]; // 公式
  std::cout << "-------巧算24点--------" << std::endl;
  std::cout << "请输入4个数：" << std::endl;
  for (int i = 0; i < NUM; i++) // 输入4个数
  {
    char buffer[20];
    int x;
    std::cin >> x;
    a[i] = x;
    itoa(x, buffer, 10); // 整数变字符串
    eq[i] = buffer;      // std::string
  }
  if (Cal(NUM, a, eq)) // 运算成功
  {
    std::cout << "计算过程：" << eq[0] << std::endl; // 输出结果
  }
  else
  {
    std::cout << "该4个数构不成24" << std::endl;
  }

  return 0;
}