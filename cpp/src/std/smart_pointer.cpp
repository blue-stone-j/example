
#include <memory>

int main()
{
  std::unique_ptr<int> ptr;
  ptr = std::make_unique<int>(42);

  std::unique_ptr<int> ptr2;
  ptr2 = std::unique_ptr<int>(new int(42));

  return 0;
}