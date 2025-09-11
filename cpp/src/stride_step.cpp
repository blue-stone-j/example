/*
When a number n is entered, if all numbers less than n have already been entered, the program should output 0,1,2,...,n 
(but each number should be printed only once overall). And every number in [0, 50] will be input exactly once.
*/

#include <iostream>
#include <vector>

int main()
{
  const int N = 50;                     // numbers are 0..N
  std::vector<bool> seen(N + 1, false); // mark arrivals
  int nextToPrint = 0;                  // smallest not-yet-printed number

  int x;
  while (std::cin >> x)
  {
    if (x < 0 || x > N) continue; // ignore out-of-range gracefully
    seen[x] = true;

    // If we can print now, print a consecutive run starting at nextToPrint
    bool printedAny = false;
    while (nextToPrint <= N && seen[nextToPrint])
    {
      std::cout << nextToPrint << " ";
      ++nextToPrint;
      printedAny = true;
    }
    if (printedAny) std::cout << std::endl; // break lines where runs end
    if (nextToPrint > N) break;             // done printing all numbers
  }
  return 0;
}
