/*
This program demonstrates how to remove elements from one vector
that are present in another vector using a hash set for fast lookup.
*/

#include <vector>
#include <unordered_set>
#include <algorithm>

int main()
{
  // if vectors isn't sorted
  {
    std::vector<int> A = {1, 2, 3, 4, 5};
    std::vector<int> B = {3, 5, 7};

    // Put all elements of B into a hash set for fast lookup
    std::unordered_set<int> setB(B.begin(), B.end());

    // Remove elements in A that exist in setB
    A.erase(std::remove_if(A.begin(), A.end(),
                           [&](int x) { return setB.count(x) > 0; }),
            A.end());

    // A now contains {1, 2, 4}
  }

  // if vectors are sorted
  {
    std::vector<int> A = {1, 2, 3, 4, 5};
    std::vector<int> B = {3, 5, 7};

    std::vector<int> result;
    std::set_difference(A.begin(), A.end(),
                        B.begin(), B.end(),
                        std::back_inserter(result));

    A.swap(result); // Now A = {1, 2, 4}
  }

  // if vectors are sorted with custom comparator
  {
    auto compare = [](const int &a, const int &b) {
      return a < b;
    };

    std::vector<int> A = {1, 2, 3, 4, 5};
    std::vector<int> B = {3, 5, 7};
    std::vector<int> result;
    std::set_difference(A.begin(), A.end(),
                        B.begin(), B.end(),
                        std::back_inserter(result), compare);
  }
}
