#include <iostream>

void inv(int *a, int length)
{
  int temp;
  int i, j;
  for (i = 0; i <= (length - 1) / 2; i++)
  {
    j    = length - 1 - i;
    temp = a[i];
    a[i] = a[j];
    a[j] = temp;
  }
}
int main()
{
  int memo[10] = {3, 4, 5, 6, 7, 8, 9, 9};
  std::cout << "原内存数据：";
  for (int i = 0; i < 10; i++)
  {
    std::cout << memo[i];
  }
  std::cout << std::endl;
  inv(memo, 10); // 反转

  std::cout << "反转后的内存数据：";
  for (int i = 0; i < 10; i++)
  {
    std::cout << memo[i];
  }
  std::cout << std::endl;
  return 0;
}