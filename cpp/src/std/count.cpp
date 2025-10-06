#include <iostream>
#include <vector>
#include <algorithm> // for std::count

struct Point
{
  int x, y;
  // Define equality
  bool operator==(const Point &other) const
  {
    return x == other.x && y == other.y;
  }
};

int main()
{
  { // Count occurrences of an integer in a vector
    std::vector<int> v = {1, 2, 3, 2, 4, 2, 5};
    int target         = 2;

    std::size_t count = std::count(v.begin(), v.end(), target);

    std::cout << "Value " << target << " appears " << count << " times.\n";
  }

  { // Count occurrences of a custom struct in a vector
    std::vector<Point> points = {{1, 2}, {3, 4}, {1, 2}, {5, 6}};
    Point target{1, 2};

    std::size_t count = std::count(points.begin(), points.end(), target);

    std::cout << "Point(" << target.x << "," << target.y
              << ") appears " << count << " times.\n";
  }

  { // Using a lambda to count occurrences based on custom criteria
    std::vector<Point> points = {{1, 2}, {3, 4}, {1, 2}, {5, 6}};
    Point target{1, 2};

    std::size_t count = std::count_if(points.begin(), points.end(),
                                      [&target](const Point &p) {
                                        return p.x == target.x && p.y == target.y;
                                      });

    std::cout << "Point(" << target.x << "," << target.y
              << ") appears " << count << " times.\n";
  }

  return 0;
}
