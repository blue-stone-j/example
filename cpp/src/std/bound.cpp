/*
find the elememts within a radius in 1D sorted data
*/

#include <vector>
#include <iostream>
#include <algorithm> // for lower_bound, upper_bound

struct Element
{
  float distance;
  std::string name;
  Element(float d, std::string n = "") :
    distance(d), name(n) {}
};

bool compare(const Element &a, const Element &b)
{
  return a.distance < b.distance;
}

bool ascending(const Element &a, const Element &b)
{
  return a.distance < b.distance;
}
bool descending(const Element &a, const Element &b)
{
  return a.distance > b.distance;
}
std::vector<Element> radiusSearch1D_sorted(const std::vector<Element> &sorted_data, float query, float radius)
{
  float lower = query - radius;
  float upper = query + radius;

  // Get iterators to the range
  /*
  add compare if element of sorted_data is complicate.
  */
  // lower_bound returns an iterator to the first element not less than (>=) the given value.
  auto lower_it = --std::lower_bound(sorted_data.begin(), sorted_data.end(), lower, compare);
  // upper_bound returns an iterator to the first element greater than (>) the given value.
  auto upper_it = std::upper_bound(sorted_data.begin(), sorted_data.end(), upper, compare);

  // Return the subrange
  return std::vector<Element>(lower_it, upper_it);
}

int main()
{
  std::vector<Element> sorted_points = {1.2, 3.4, 5.6, 5.9, 6.1, 7.8, 10.0};
  float query                        = 6.0;
  float radius                       = 1.0;

  if (!std::is_sorted(sorted_points.begin(), sorted_points.end(), ascending))
  {
    // Assuming it is sorted in descending order
    std::reverse(sorted_points.begin(), sorted_points.end());
  }

  std::vector<Element> result = radiusSearch1D_sorted(sorted_points, query, radius);

  std::cout << "Points within radius: ";
  for (const auto &r : result)
  {
    std::cout << r.distance << " " << r.name;
  }
  std::cout << std::endl;

  return 0;
}
