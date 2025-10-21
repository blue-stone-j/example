/*
compare std::set and std::unordered_set
*/

#include <set>
#include <unordered_set>

int main(int argc, char **argv)
{
  struct Descending
  {
    bool operator()(int a, int b) const { return a > b; }
  };
  std::set<int, Descending> s = {1, 3, 2}; // stored as 3, 2, 1

  struct Point
  {
    int x, y;
  };

  struct PointHash
  {
    std::size_t operator()(const Point &p) const
    {
      return std::hash<int>()(p.x) ^ (std::hash<int>()(p.y) << 1);
    }
  };

  struct PointEqual
  {
    bool operator()(const Point &a, const Point &b) const
    {
      return a.x == b.x && a.y == b.y;
    }
  };

  std::unordered_set<Point, PointHash, PointEqual> points;

  return 0;
}
