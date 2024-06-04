// 求最长字符
#include <cctype>
#include <iostream>
#include <cstring>
#define M 1000
const char *extra = "$";
int main()
{
  char *ch       = new char[M];
  int start      = 0;              // 最长字符的开始
  int left       = 0;              // 每段字符的左边值
  int length     = 0;              // 当前字符串的长度
  int length_max = 0;              // 最长字符的长度
  std::cin >> ch;                  // 输入字符串
  if (isalpha(ch[strlen(ch) - 1])) // 是字母
  {
    // 加一个不是字母的元素，以找到最后一个字符子串
    strcat(ch, extra);
  }
  for (size_t i = 0; i < strlen(ch); i++)
  {
    if (isalpha(ch[i])) // 是字母
    {
      length++; // 当前字符串长度加1
    }
    else // 不是字母
    {
      if (length > length_max) // 如果当前字符串的长度大于最大字符的长度
      {                        // 更新开始
        start      = left;
        length_max = length;
      }
      left   = i + 1; // 新的左边值
      length = 0;     // 新的字符串长度从0开始增加
    }
  }
  std::cout << "最长字符串：";
  for (int i = start; i < start + length_max; i++)
  {
    std::cout << ch[i];
  }
  std::cout << std::endl;
  delete ch;
  ch = NULL;

  return 0;
}