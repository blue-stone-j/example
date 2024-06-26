#include <iostream>
#include <cstring>

#define M 10
#define N 10
class Maze
{
 public:
  Maze();                          // 构造函数
  ~Maze() {}                       // 析构函数
  void PrintMaze();                // 打印迷宫
  void GetPos();                   // 获取出入口位置
  void SearchMaze();               // 获取迷宫路径
  bool EvaluateMaze(int i, int j); // 判断当前点是否可行
 private:
  int maze[M][N]; // 迷宫
  int start_row;  // 入口行
  int start_col;  // 入口列
  int end_row;    // 出口行
  int end_col;    // 出口列
  bool succeed;   // 是否找到出口
};
Maze::Maze()
{
  int copy[M][N] = {{1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                    {1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
                    {1, 0, 1, 0, 1, 0, 1, 0, 0, 1},
                    {1, 0, 0, 1, 0, 1, 1, 1, 0, 1},
                    {1, 1, 0, 1, 0, 1, 1, 0, 0, 0},
                    {1, 0, 0, 0, 0, 0, 1, 0, 0, 1},
                    {1, 1, 1, 1, 1, 1, 1, 0, 1, 1},
                    {1, 1, 0, 1, 0, 1, 1, 0, 0, 1},
                    {1, 0, 0, 0, 0, 0, 1, 1, 0, 1},
                    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};
  memcpy(maze, copy, sizeof(maze)); // 迷宫初始化,0为可行，1为不可行
  succeed = false;                  // 初始化为没有
}
void Maze::PrintMaze()
{
  std::cout << "显示迷宫10*10：" << std::endl;
  for (int i = 0; i < M; i++) // 行
  {
    for (int j = 0; j < N; j++) // 列
    {
      if (maze[i][j] == 1) // 不可行
      {
        std::cout << "1";
      }
      else // 可行
      {
        std::cout << "0";
      }
    }
    std::cout << std::endl; // 下一行
  }
}
void Maze::GetPos()
{
  std::cout << "请输入迷宫入口行：";
  std::cin >> start_row;
  std::cout << "请输入迷宫入口列：";
  std::cin >> start_col;
  std::cout << "请输入迷宫出口行：";
  std::cin >> end_row;
  std::cout << "请输入迷宫出口列：";
  std::cin >> end_col;
}
void Maze::SearchMaze()
{
  if (EvaluateMaze(start_row, start_col) == false) // 没有找到出口
  {
    std::cout << std::endl
              << "没有找到出口\n";
  }
  else // 找到
  {
    std::cout << std::endl
              << "显示路径：" << std::endl;
    for (int i = 0; i < M; i++) // 行
    {
      for (int j = 0; j < N; j++) // 列
      {
        if (maze[i][j] == 1) // 打印不可通行位置
        {
          std::cout << "1";
        }
        else if (maze[i][j] == 2) // 打印路径
        {
          std::cout << "*";
        }
        else // 可通行的但不是路径为空格
        {
          std::cout << " ";
        }
      }
      std::cout << std::endl;
    }
  }
}
bool Maze::EvaluateMaze(int i, int j)
{
  maze[i][j] = 2;                   // 与迷宫已被判断的元素区别
  if (i == end_row && j == end_col) // 搜索到出口
  {
    succeed = true;
  }
  if (succeed != true && maze[i][j + 1] == 0) // 向后一列可通行的
  {
    EvaluateMaze(i, j + 1);
  }
  if (succeed != true && maze[i + 1][j] == 0) // 向后一行可通行的
  {
    EvaluateMaze(i + 1, j);
  }
  if (succeed != true && maze[i][j - 1] == 0) // 向前一列可通行的
  {
    EvaluateMaze(i, j - 1);
  }
  if (succeed != true && maze[i - 1][j] == 0) // 向前一行可通行的
  {
    EvaluateMaze(i - 1, j);
  }

  if (succeed != true) // 重新赋为可通行的
  {
    maze[i][j] = 0;
  }
  return succeed;
}

int main()
{
  // int i,j;//循环变量
  Maze _maze;         // 迷宫对象
  _maze.PrintMaze();  // 打印迷宫
  _maze.GetPos();     // 获取出入口位置
  _maze.SearchMaze(); // 寻找出口路径

  return 0;
}