#include <iostream>

void add()
{
  std::cout << "2+3=" << 2 + 3 << std::endl;
}

void substract()
{
  std::cout << "2-3=" << 2 - 3 << std::endl;
}
void add1()
{
  std::cout << "2+3=" << 2 + 3 << std::endl;
}

void sub1()
{
  std::cout << "2-3=" << 2 - 3 << std::endl;
}
int main()
{
  atexit(add);
  atexit(substract);
  atexit(add1);
  atexit(sub1);
  std::cout << "main中" << std::endl;

  return 0;
}