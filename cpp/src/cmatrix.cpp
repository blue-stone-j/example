#include <iostream>

class CMatrix
{
 public:
  // 构造函数
  CMatrix() // 构造函数
  {
    m_nRow  = 0;
    m_nCol  = 0;
    m_pData = NULL; // 空值
  }
  CMatrix(int nRow, int nCol, int *pAry) // 带参构造函数
  {
    m_nRow = nRow;
    m_nCol = nCol;

    m_pData = new int *[m_nRow]; // 二维内存初始化
    for (int i = 0; i < m_nRow; i++)
    {
      m_pData[i] = new int[nCol];
      for (int j = 0; j < m_nCol; j++)
      {
        m_pData[i][j] = pAry[i * m_nCol + j]; // 赋值
      }
    }
  }
  CMatrix(CMatrix &src) // 拷贝构造函数
  {
    m_nRow = src.Row(); // 获取行
    m_nCol = src.Col(); // 获取列

    m_pData = new int *[src.Row()];
    for (int i = 0; i < src.Row(); i++) // 行
    {
      m_pData[i] = new int[src.Col()];
      for (int j = 0; j < src.Col(); j++) // 列
      {
        m_pData[i][j] = src.m_pData[i][j]; // 赋值
      }
    }
  }
  void output()
  {
    for (int i = 0; i < m_nRow; i++) // 行
    {
      for (int j = 0; j < m_nCol; j++) // 列
      {
        std::cout << m_pData[i][j] << " "; // 输入元素
      }
      std::cout << std::endl;
    }
  }
  ~CMatrix() // 析构函数
  {
    if (m_pData != NULL) // 如果不为空，释放
    {
      for (int i = 0; i < m_nRow; i++) // 释放每行
      {
        delete[] m_pData[i];
      }
      delete[] m_pData;
      m_pData = NULL;
    }
    m_nRow = 0; // 行列值回0
    m_nCol = 0;
  }

  int Row() const { return m_nRow; } // 获取行
  int Col() const { return m_nCol; } // 获取列
 private:
  int m_nRow;    // 行数
  int m_nCol;    // 列数
  int **m_pData; // 二维矩阵
};

int main()
{
  int n, m;  // 行列
  int *data; // 矩阵内存
  std::cout << "请输入矩阵的行数：";
  std::cin >> n; // 行
  std::cout << "请输入矩阵的列数：";
  std::cin >> m;         // 列
  data = new int[n * m]; // 申请内存
  std::cout << "请初始化" << n << "行" << m << "列矩阵元素" << std::endl;
  for (int i = 0; i < n; i++)
  {
    for (int j = 0; j < m; j++)
    {
      std::cin >> data[i * m + j]; // 初始化
    }
  }
  CMatrix m1(n, m, data); // 声明类对象1
  CMatrix m2(m1);         // 声明类对象2
  std::cout << "使用拷贝构造函数后，m2对象的矩阵为" << std::endl;
  m2.output(); // 输出

  return 0;
}