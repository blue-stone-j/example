/*

*/

#include <algorithm>
#include <iostream>

int main(int argc, char **argv)
{
  {
    /* constexpr const T& clamp(const T& v, const T& lo, const T& hi, Compare comp);
     No-throw guarantee
     Returns reference to one of the arguments (v, lo, or hi)
    */
    int x1 = std::clamp(5, 1, 10);  // within range → 5
    int x2 = std::clamp(-3, 1, 10); // below min → 1
    int x3 = std::clamp(15, 1, 10); // above max → 10

    std::cout << x1 << ' ' << x2 << ' ' << x3 << '\n';
  }

  return 0;
}