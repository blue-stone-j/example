#include <algorithm>
#include <iostream>

int capacity    = 30;
int weights[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
int values[10]  = {3, 9, 1, 8, 4, 6, 7, 2, 5};

int cnt = 0;

int reverse(int value = 0, int weight = 0, int i = 0)
{
  // std::cout << ++cnt << std::endl;
  if (i == 10)
  {
    return value;
  }
  if (weight + weights[i] > capacity)
  {
    return value;
  }
  value = std::max(reverse(value, weight, i + 1), reverse(value + values[i], weight + weights[i], i + 1));

  return value;
}

int main()
{
  std::cout << "--------- 01 ----------" << std::endl;
  std::cout << reverse() << std::endl;

  int res[11][30] = {{0}};
  for (int j = 1; j < 30; j++)
  {
    for (int i = 0; i < 10; i++)
    {
      if (j < weights[i])
      {
        res[i + 1][j] = res[i][j]; // can't put this item into bag
      }
      else
      {
        res[i + 1][j] = std::max(res[i][j - weights[i]] + values[i],
                                 res[i][j]);
      }
    }
  }

  std::cout << res[10][29] << std::endl;

  std::cout << "----------infinite---------" << std::endl;

  int max_j[31] = {0};
  for (int j = 1; j < 31; j++)
  {
    for (int i = 0; i < 10; i++)
    {
      if (j < weights[i])
      {
        continue;
      }
      if (max_j[j] < max_j[j - weights[i]] + values[i])
      {
        max_j[j] = max_j[j - weights[i]] + values[i];
        std::cout << j << "-" << i << ": " << max_j[j] << std::endl;
      }
    }
  }

  return 0;
}