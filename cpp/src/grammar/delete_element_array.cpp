#include <iostream>
#include <cstring>

// 利用指针删除数组中的指定元素（指针移动）
int main()
{
  char ch[]     = "ajfdklsafj";
  char *pointer = ch;                     // 获取ch的首地址
  pointer += strlen(ch) / 2;              // 向后移动strlen(ch)/2位
  for (size_t i = 3; i < strlen(ch); i++) // 删除第4个元素
  {
    ch[i] = ch[i + 1];
  }
  std::cout << ch << std::endl;
  return 0;
}