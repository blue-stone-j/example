// Description: Distribute blocks of data evenly into a given number of strips.

#include <iostream>
#include <vector>

void distributeBlocksEvenly(const std::vector<std::vector<int>> &blocks, std::vector<std::vector<int>> &strips, int strip_num)
{
  // Clear existing content in strips
  strips.clear();
  strips.resize(strip_num);

  for (const auto &block : blocks)
  { // Iterate over each block
    int total_size = block.size();
    int base_size  = total_size / strip_num;
    int remainder  = total_size % strip_num; // Extra elements to distribute

    int start = 0;
    for (int i = 0; i < strip_num; i++)
    {
      // The first `remainder` strips receive one extra element
      int end = start + base_size + (i < remainder ? 1 : 0);
      strips[i].insert(strips[i].end(), block.begin() + start, block.begin() + end);
      start = end; // Move to the next section
    }
  }
}

// Function to print strips
void printStrips(const std::vector<std::vector<int>> &strips)
{
  for (size_t i = 0; i < strips.size(); ++i)
  {
    std::cout << "Strip " << i << ": ";
    for (int val : strips[i])
    {
      std::cout << val << " ";
    }
    std::cout << std::endl;
  }
}

int main()
{
  // Example blocks of data
  std::vector<std::vector<int>> blocks = {
      {0, 1, 2, 3, 4},     // First block
      {5, 6, 7, 8, 9, 10}, // Second block
      {11, 12, 13, 14}     // Third block
  };

  int strip_num = 3; // Number of strips to distribute data into
  std::vector<std::vector<int>> strips(strip_num);

  // Distribute blocks evenly into strips
  distributeBlocksEvenly(blocks, strips, strip_num);

  // Print the resulting strips
  printStrips(strips);

  return 0;
}
