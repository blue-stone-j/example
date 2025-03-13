// whether there is at least one value that exists in both set1 and set2.

#include <iostream>
#include <set>
#include <algorithm>

int main()
{
  std::set<int> set1 = {1, 2, 3, 4};
  std::set<int> set2 = {3, 4, 5, 6};

  // Create a set to hold the intersection
  std::set<int> intersection;

  // Find the intersection of set1 and set2
  std::set_intersection(set1.begin(), set1.end(), set2.begin(), set2.end(),
                        std::inserter(intersection, intersection.begin()));

  // Check if the intersection is non-empty
  if (!intersection.empty())
  {
    std::cout << "There are common elements: ";
    for (const auto &val : intersection)
    {
      std::cout << val << " ";
    }
    std::cout << std::endl;
  }
  else
  {
    std::cout << "There are no common elements." << std::endl;
  }

  return 0;
}