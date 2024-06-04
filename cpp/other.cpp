#include <iostream>
using namespace std;

void length(char ch[])
{
  cout << "字符数组字节：" << sizeof ch << endl; // 输出输入参数的字节大小
}
int main()
{
  char ch[20] = "Hello World";                   // 字符数组
  char *point;                                   // char型指针
  point = ch;                                    // 赋值
  cout << "指针字节：" << sizeof point << endl;  // 输出指针的字节大小
  cout << "字符数组字节：" << sizeof ch << endl; // 输出字符数组所占字节
  length(ch);                                    // 调用length()函数
  return 0;
}