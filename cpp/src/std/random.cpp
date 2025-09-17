#include <iostream>
#include <random>

int main()
{
  {
    // Create a random device and generator
    std::random_device rd;
    std::mt19937 gen(rd()); // Mersenne Twister engine

    // Define range [1, 100]
    std::uniform_int_distribution<> dist(1, 100);

    // Generate random number
    int random_num = dist(gen);
    std::cout << "Random number: " << random_num << std::endl;
  }

  {
    std::random_device rd;
    std::mt19937 gen(rd());                          // Mersenne Twister engine
    std::uniform_real_distribution<> dist(0.0, 1.0); // range [0, 1)

    double random_val = dist(gen);
    std::cout << "Random float: " << random_val << std::endl;
  }

  return 0;
}
