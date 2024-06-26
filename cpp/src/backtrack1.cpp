#include <iostream>
using namespace std;

#define N 8
int column[N + 1];      // 同栏是否有皇后，1表示有
int rup[2 * N + 1];     // 副对角线是否有皇后
int lup[2 * N + 1];     // 主对角线是否有皇后
int queen[N + 1] = {0}; // 八皇后位置
int num          = 0;   // 摆法

void backtrack(int);
void solution();

void backtrack(int i) // 求解
{
  int j;     // 循环量
  if (i > N) // 八个皇后位置都确定
  {
    num++; // 解决方法加1
  }
  else // 继续确定剩余皇后的位置
  {
    for (j = 1; j <= N; j++) // 循环
    {
      if (column[j] == 1 && rup[i + j] == 1 && lup[i - j + N] == 1) // 都有皇后
      {
        queen[i]  = j;                               // 确定皇后位置
        column[j] = rup[i + j] = lup[i - j + N] = 0; // 去掉皇后
        backtrack(i + 1);
        column[j] = rup[i + j] = lup[i - j + N] = 1; // 恢复
      }
    }
  }
}
void solution() // 显示解决方案
{
  int i, j;
  for (j = 1; j <= N; j++)
  {
    for (i = 1; i <= N; i++)
    {
      if (queen[j] == i) // 皇后位置
      {
        cout << "Q";
      }
      else // 否
      {
        cout << "—";
      }
    }
    cout << endl;
  }
}


int main()
{
  cout << "------八皇后位置摆放-----" << endl;
  int i;
  for (i = 1; i <= N; i++)
  {
    column[i] = 1;
  }
  for (i = 1; i <= 2 * N; i++)
  {
    rup[i] = lup[i] = 1;
  }
  backtrack(1); // 回溯法
  cout << "共有" << num << "种摆法" << endl;
  cout << "其中一种摆法为：" << endl;
  solution(); // 显示当前摆法

  return 0;
}