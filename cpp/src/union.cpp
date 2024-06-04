#include <iostream>

union test
{
  int aa;
  char ch;
};
int main()
{
  test _test; // 组合
  _test.aa = 0x00000001;
  std::cout << "本机的内存数据排列顺序为：";
  if (_test.ch == 1)
  {
    std::cout << "先低后高" << std::endl;
  }
  else if (_test.ch == 0)
  {
    std::cout << "先高后低" << std::endl;
  }

  return 0;
}