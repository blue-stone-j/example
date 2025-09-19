#include <thread>
#include <chrono>

int main(int argc, char **argv)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 0.5 sec
  std::this_thread::sleep_for(std::chrono::seconds(1));        // 1 sec
  std::this_thread::sleep_for(std::chrono::minutes(2));        // 2 minutes

  return 0;
}