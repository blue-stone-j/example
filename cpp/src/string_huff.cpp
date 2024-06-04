#include <iostream>
#include <string>

struct huffTree
{
  int parent;       // 父亲
  int lchild;       // 左孩子
  int rchild;       // 右孩子
  int weight;       // 权重
  std::string flag; // 标志
};

struct Lowest_Node // 第0级节点的字符与频度
{
  char ch;
  int ch_num;
};
// 确定每个字符的huffman编码,输出参数为a、b,最小的两个节点的位置
void coding(int length, huffTree tree[], int n, int &a, int &b)
{
  int i;
  int r, s;
  r = s = length; // 节点个数最大不会超过字符串的长度

  // 从所有节点中找到最小的
  for (i = 0; i < n; i++)
  {
    if ((tree[i].weight < r) && (tree[i].parent == -1))
    {
      r = tree[i].weight;
      a = i;
    }
  }

  // 从所有节点中找到次小的
  for (i = 0; i < n; i++)
  {
    if ((tree[i].weight < s) && (i != a) && (tree[i].parent == -1))
    {
      s = tree[i].weight;
      b = i;
    }
  }
}

// 计算每个字符出现的频度并排序
void frequency(std::string str)
{
  int length        = str.length();            // 长度
  Lowest_Node *node = new Lowest_Node[length]; // 声明最0级节点

  int i, j; // 循环因子
  for (i = 0; i < length; i++)
  {
    node[i].ch_num = 0; // 初始化频度
  }

  int char_type_num = 0;       // 初始为0种字符
  for (i = 0; i < length; i++) // 循环整个字符串
  {
    for (j = 0; j < char_type_num; j++)
    {
      if (str[i] == node[j].ch || (node[j].ch >= 'a' && node[j].ch <= 'z' && str[i] + 32 == node[j].ch))
      {
        break; // 该字符没有出现过，跳出循环
      }
    }

    if (j < char_type_num) // 该字符重复出现，对应的记数器加1
    {
      node[j].ch_num++;
    }
    else // 新出现的字符，记录到ch[j]中，对应计数器加1
    {
      if (str[i] >= 'A' && str[i] <= 'Z')
      {
        node[j].ch = str[i] + 32;
      }
      else
      {
        node[j].ch = str[i];
      }
      node[j].ch_num++;
      char_type_num++; // 字符的种类数加1
    }
  }

  // 按频度从大到小排序
  for (i = 0; i < char_type_num; i++)
  {
    for (j = i; j < char_type_num; j++)
    {
      if (node[j].ch_num < node[j + 1].ch_num) // 如果前一个小于后一个，交换
      {
        int temp;     // 临时频度
        char ch_temp; // 临时字符
        temp               = node[j].ch_num;
        ch_temp            = node[j].ch;
        node[j].ch_num     = node[j + 1].ch_num;
        node[j].ch         = node[j + 1].ch;
        node[j + 1].ch_num = temp;
        node[j + 1].ch     = ch_temp;
      }
    }
  }

  for (i = 0; i < char_type_num; i++) // 打印字符频度
  {
    std::cout << "字符" << node[i].ch << "出现了" << node[i].ch_num << "次" << std::endl;
  }

  huffTree *huff = new huffTree[2 * char_type_num - 1]; // 此变量的声明需位于确定char_type_num值后
  huffTree temp;
  std::string *code = new std::string[2 * char_type_num - 1]; // 存放各个字符的编码

  for (i = 0; i < 2 * char_type_num - 1; i++) // 节点初始化
  {
    huff[i].lchild = -1;
    huff[i].parent = -1;
    huff[i].rchild = -1;
    huff[i].flag   = -1;
  }

  // 将排序后的第0级节点权重赋给树节点
  for (j = 0; j < char_type_num; j++)
  {
    huff[j].weight = node[j].ch_num;
  }

  // 所有节点中最小的两个节点的位置
  int min1 = 0, min2 = 0;
  // 赋值0级之上的节点
  for (int k = char_type_num; k < 2 * char_type_num - 1; k++)
  {
    coding(length, huff, k, min1, min2);
    huff[min1].parent = k;
    huff[min2].parent = k;
    huff[min1].flag   = "0";
    huff[min2].flag   = "1";
    huff[k].lchild    = min1;
    huff[k].rchild    = min2;
    huff[k].weight    = huff[min1].weight + huff[min2].weight;
  }

  // 计算每个字符的huff编码
  for (i = 0; i < char_type_num; i++)
  {
    temp = huff[i];
    while (1)
    {
      code[i] = temp.flag + code[i];
      temp    = huff[temp.parent];
      if (temp.parent == -1)
      {
        break;
      }
    }
  }
  std::cout << "字符串的每个字符huffman编码为:" << std::endl;
  for (i = 0; i < char_type_num; i++)
  {
    std::cout << node[i].ch << "  " << code[i] << std::endl;
  }

  std::cout << "整个字符串的huffman编码为：" << std::endl;
  for (i = 0; i < length; i++)
  {
    // 查找这个字符的编码
    for (j = 0; j < char_type_num; j++)
    {
      if (str[i] == node[j].ch)
      {
        std::cout << code[j];
      }
    }
  }
  std::cout << std::endl;

  // 释放内存
  delete[] node;
  node = NULL;
  delete[] huff;
  huff = NULL;
  delete[] code;
  code = NULL;
}

int main()
{
  std::string str; // 目标字符串
  std::cout << "请输入一个字符串:";
  std::cin >> str;
  frequency(str); // 求各个元素的频度

  return 0;
}