// Find the index of the max element in a container

#include <iostream>
#include <vector>
#include <algorithm>

int main()
{
  // vector
  {
    std::vector<int> vec = {3, 7, 2, 9, 5, 9, 1};

    // Find index of max element
    auto max_it   = std::max_element(vec.begin(), vec.end());
    int max_index = std::distance(vec.begin(), max_it);

    std::cout << "Max value: " << *max_it << " at index " << max_index << std::endl;
  }

  // raw array
  {
    int arr[] = {3, 7, 2, 9, 5, 9, 1};

    // Find index of max element
    auto max_it   = std::max_element(std::begin(arr), std::end(arr));
    int max_index = std::distance(std::begin(arr), max_it);

    std::cout << "Max value: " << *max_it << " at index " << max_index << std::endl;
  }

  // raw array: for C++98/03 without std::begin and std::end
  {
    int arr[]     = {3, 7, 2, 9, 5, 9, 1};
    int *max_it   = std::max_element(arr, arr + sizeof(arr) / sizeof(arr[0]));
    int max_index = max_it - arr;

    std::cout << "Max value: " << *max_it << " at index " << max_index << std::endl;
  }

  return 0;
}
