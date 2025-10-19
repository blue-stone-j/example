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

  // judge whether it contains specific value
  {
    int valueToFind = 4;
    if (std::find(vec.begin(), vec.end(), valueToFind) != vec.end())
    {
      std::cout << "Vector contains " << valueToFind << std::endl;
    }

    // if vector is sorted
    if (std::binary_search(vec.begin(), vec.end(), valueToFind))
    {
      std::cout << "Found\n";
    }
  }

  // merge two std::vector and one of the std::vectors will not be used anymore
  {
    std::vector<int> v1 = {1, 2, 3};
    std::vector<int> v2 = {4, 5, 6};

    v1.insert(v1.end(), std::make_move_iterator(v2.begin()), std::make_move_iterator(v2.end()));
    /*
    After the move, each element of v2 is in a valid but unspecified state (for int or bool this just means copied, 
    for std::string it usually becomes empty, for complex types it depends on their move semantics).
    */
  }

  return 0;
}
