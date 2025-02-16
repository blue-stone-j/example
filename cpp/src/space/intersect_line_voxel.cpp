/*
Judge whether a line intersects with a voxel, whose sides is parallel with coordinate axis
*/

#include <iostream>

struct Point
{
  float x, y, z;
  Point(float x_, float y_, float z_) :
    x(x_), y(y_), z(z_) {}
  Point operator-(Point b) const
  {
    return Point(this->x - b.x, this->y - b.y, this->z - b.z);
  }
  float operator[](int i) const
  {
    switch (i)
    {
      case 0: {
        return x;
      }
      case 1: {
        return y;
      }
      case 2: {
        return z;
      }
      default: {
        return 0;
      }
    }
  }
};

bool doesLineSegmentIntersectNode(
    const Point &line_start,
    const Point &line_end,
    const Point &min_bound,
    const Point &max_bound)
{
  float tmin = 0.0f;
  float tmax = 1.0f;

  Point direction = line_end - line_start;

  for (int i = 0; i < 3; ++i)
  { // Check X, Y, Z axes
    if (std::abs(direction[i]) < 1e-6)
    { // Parallel line case
      if (line_start[i] < min_bound[i] || line_start[i] > max_bound[i])
        return false; // Line is outside this axis range
    }
    else
    {
      float inv_d = 1.0f / direction[i];
      float t1    = (min_bound[i] - line_start[i]) * inv_d;
      float t2    = (max_bound[i] - line_start[i]) * inv_d;

      if (t1 > t2) std::swap(t1, t2);

      tmin = std::max(tmin, t1);
      tmax = std::min(tmax, t2);

      if (tmin > tmax)
        return false; // No intersection
    }
  }

  return true; // Intersection exists
}

int main()
{
  // Point line_start(0.0f, 0.0f, 0.0f);
  // Point line_end(1.0f, 1.0f, 1.0f);
  Point line_start(100, 100, 100);
  Point line_end(10, 10, 10);

  Point min_bound(-5, -5, -5), max_bound(5, 5, 5);

  if (doesLineSegmentIntersectNode(line_start, line_end, min_bound, max_bound))
  {
    std::cout << "Line intersects the node!" << std::endl;
  }
  else
  {
    std::cout << "Line does not intersect the node." << std::endl;
  }

  return 0;
}
