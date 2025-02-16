/*
a std::set storing points and each point has a small range of influence that
"expands" toward its neighbors, expansion distance is 5.
*/

#include <algorithm>
#include <iostream>
#include <set>
#include <vector>

struct Range
{
  double start;
  double end;
};

// Helper function to merge overlapping ranges
std::vector<Range> mergeRanges(std::vector<Range> &ranges)
{
  std::vector<Range> merged;
  if (ranges.empty())
    return merged;

  // Sort ranges by start point
  std::sort(ranges.begin(), ranges.end(),
            [](const Range &a, const Range &b) { return a.start < b.start; });

  // Merge overlapping ranges
  Range current = ranges[0];
  for (size_t i = 1; i < ranges.size(); ++i)
  {
    if (ranges[i].start <= current.end)
    {
      current.end = std::max(current.end, ranges[i].end); // Extend the range
    }
    else
    {
      merged.push_back(current);
      current = ranges[i];
    }
  }
  merged.push_back(current);

  return merged;
}

// Calculates the left and right range of influence based on neighbors and the
// distance threshold.
void computeRanges(const std::set<double> &points, double threshold,
                   std::vector<Range> &ranges)
{
  auto it   = points.begin();
  auto prev = it;
  ++it;

  for (; it != points.end(); ++it)
  {
    // Compute range for the current point
    double left_dist =
        (it == points.begin()) ? threshold : std::min(*it - *prev, threshold);
    double right_dist = (std::next(it) == points.end()) ? threshold : std::min(*std::next(it) - *it, threshold);

    // Calculate range for current point
    ranges.push_back({*it - left_dist, *it + right_dist});
    prev = it;
  }
}

double calculateCoveredLength(const std::vector<Range> &mergedRanges)
{
  double totalCovered = 0.0;
  for (const auto &range : mergedRanges)
  {
    totalCovered += range.end - range.start;
  }
  return totalCovered;
}

int main()
{
  // Example points
  std::set<double> points = {1.0, 6.0, 11.0, 16.0}; // Sparse points
  double threshold        = 5.0;
  double segmentStart = 0.0, segmentEnd = 20.0;

  // Compute ranges
  std::vector<Range> ranges;
  computeRanges(points, threshold, ranges);

  // Merge overlapping ranges
  std::vector<Range> mergedRanges = mergeRanges(ranges);

  // Calculate total covered length
  double totalCovered = calculateCoveredLength(mergedRanges);

  // Calculate coverage proportion
  double segmentLength      = segmentEnd - segmentStart;
  double coverageProportion = totalCovered / segmentLength;

  // Output results
  std::cout << "Total covered length: " << totalCovered << "\n";
  std::cout << "Coverage proportion: " << coverageProportion * 100.0 << "%\n";

  return 0;
}
