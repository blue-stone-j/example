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
      std::string exp_a, exp_b;
      a = number[i];
      b = number[j];
      // 挪动后面有效数字
      number[j]   = number[n - 1];
      exp_a       = equation[i];
      exp_b       = equation[j];
      equation[j] = equation[n - 1];
      // a+b
      equation[i] = '(' + exp_a + '+' + exp_b + ')';
      number[i]   = a + b;
      if (Cal(n - 1, number, equation))
      {
        return true;
      }
      // a-b
      equation[i] = '(' + exp_a + '-' + exp_b + ')';
      number[i]   = a - b;
      if (Cal(n - 1, number, equation))
      {
        return true;
      }
      // b-a
      equation[i] = '(' + exp_b + '-' + exp_a + ')';
      number[i]   = b - a;
      if (Cal(n - 1, number, equation))
      {
        return true;
      }
      //(a*b)
      equation[i] = '(' + exp_a + '*' + exp_b + ')';
      number[i]   = a * b;
      if (Cal(n - 1, number, equation))
      {
        return true;
      }
      //(a/b)
      if (b != 0) // 除数不为0
      {
        equation[i] = '(' + exp_a + '/' + exp_b + ')';
        number[i]   = a / b;
        if (Cal(n - 1, number, equation))
        {
          return true;
        }
      }
      //(b/a)
      if (a != 0) // 除数不为0
      {
        equation[i] = '(' + exp_b + '/' + exp_a + ')';
        number[i]   = b / a;
        if (Cal(n - 1, number, equation))
        {
          return true;
        }
      }
      // 恢复
      number[i]   = a;
      number[j]   = b;
      equation[i] = exp_a;
      equation[j] = exp_b;
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
    int x;
    std::cin >> x;
    a[i]  = x;
    eq[i] = std::to_string(x);
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