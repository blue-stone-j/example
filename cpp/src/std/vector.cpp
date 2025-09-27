// This program demonstrates how to delete elements from a vector evenly until it reaches a target size.

#include <algorithm>
#include <iostream>
#include <vector>

void deleteEvenly(std::vector<int> &vec, std::size_t targetSize)
{
  while (vec.size() > targetSize)
  {
    std::size_t removeCount = vec.size() - targetSize;
    std::size_t step        = vec.size() / removeCount; // Calculate step size

    for (std::size_t i = 0; i < removeCount; ++i)
    {
      std::size_t index = i * step;
      if (index >= vec.size()) break;
      vec.erase(vec.begin() + index);
    }
  }
}

int main()
{
  std::vector<int> vec = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

  deleteEvenly(vec, 5); // Reduce the vector size to 5 elements

  // Print the remaining elements
  for (int num : vec)
  {
    std::cout << num << " ";
  }
  std::cout << std::endl;

  // filter vector in place
  vec.erase(std::remove_if(vec.begin(), vec.end(), [](int x) { return x % 2 != 0; }), // remove odd
            vec.end());

  return 0;
}
