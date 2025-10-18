/*
1. std::iota is a function in the C++ Standard Library (defined in <numeric>) that fills a range with sequentially 
             increasing values, starting from a specified initial value.
*/

#include <iostream>
#include <vector>
#include <numeric> // for std::iota

int main()
{
  std::vector<int> v(5);

  // Fill v with 10, 11, 12, 13, 14
  std::iota(v.begin(), v.end(), 10);

  for (int n : v)
  { std::cout << n << ' '; }
  std::cout << '\n';
}