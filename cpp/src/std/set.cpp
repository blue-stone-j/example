/*
compare std::set and std::unordered_set
*/

#include <set>
#include <unordered_set>
#include <iostream>

int main(int argc, char **argv)
{
  // std::set with custom comparator
  {
    struct Descending
    {
      //C++14’s transparent comparator to avoid constructing CustomType manually
      using is_transparent = void; // enables heterogeneous lookup
      bool operator()(int a, int b) const { return a > b; }
    };
    std::set<int, Descending> s = {1, 3, 2}; // stored as 3, 2, 1
  }

  // std::unordered_set with custom hash and equality
  {
    struct Point
    {
      int x, y;
    };

    struct PointHash
    {
      using is_transparent = void; // enables heterogeneous lookup
      std::size_t operator()(const Point &p) const
      {
        return std::hash<int>()(p.x) ^ (std::hash<int>()(p.y) << 1);
      }
    };

    struct PointEqual
    {
      using is_transparent = void;
      bool operator()(const Point &a, const Point &b) const
      {
        return a.x == b.x && a.y == b.y;
      }
    };

    std::unordered_set<Point, PointHash, PointEqual> points;
  }

  // Finding an element in std::set with custom type
  {
    struct CustomType
    {
      int id;
      std::string name;

      bool operator<(const CustomType &other) const
      {
        return id < other.id;
      }
    };
    std::set<CustomType> s = {
        {1, "Alice"},
        {2, "Bob"},
        {3, "Charlie"}};

    int target_id = 2;

    // Create a temporary object with the same id
    CustomType temp{target_id, ""};

    auto it = s.find(temp);
    if (it != s.end())
    {
      std::cout << "Found: " << it->id << ", " << it->name << std::endl;
    }
    else
    {
      std::cout << "Not found\n";
    }
  }

  return 0;
}
