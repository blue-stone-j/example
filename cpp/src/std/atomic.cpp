

/* The issue is that std::atomic is neither copyable nor movable, while std::vector::resize() (when it grows) requires 
   the element type to be either copy-constructible or move-constructible so it can fill new slots. Since 
   std::atomic<bool> only has a default constructor and is not copyable/movable, resize() is not allowed.
*/

#include <atomic>
#include <vector>
#include <iostream>

struct AtomicBool
{
  std::atomic<bool> value;

  // Constructors
  AtomicBool() noexcept :
    value(false) {}
  explicit AtomicBool(bool v) noexcept :
    value(v) {}

  // Copy constructor and assignment — allowed for convenience
  AtomicBool(const AtomicBool &other) noexcept
  {
    value.store(other.value.load(std::memory_order_relaxed),
                std::memory_order_relaxed);
  }

  AtomicBool &operator=(const AtomicBool &other) noexcept
  {
    if (this != &other)
    {
      value.store(other.value.load(std::memory_order_relaxed),
                  std::memory_order_relaxed);
    }
    return *this;
  }

  // Assignment from bool
  AtomicBool &operator=(bool v) noexcept
  {
    value.store(v, std::memory_order_relaxed);
    return *this;
  }

  // Implicit conversion to bool
  operator bool() const noexcept
  {
    return value.load(std::memory_order_relaxed);
  }
};

class Foo
{
 public:
  std::vector<AtomicBool> flags;

  Foo(std::size_t n, bool initial = false) :
    flags(n, AtomicBool(initial)) {}
};

int main(int argc, char **argv)
{
  std::vector<std::atomic<bool>> v(10); // 10 default-constructed atomics (initially false)
  for (auto &x : v)
  {
    x.store(true); // initialize to true
  }
  for (auto &x : v)
  {
    std::cout << x.load() << " ";
  }

  std::cout << std::endl;

  Foo f(5, true);

  // Works like normal bool vector
  f.flags[2] = false;

  for (auto &x : f.flags)
  {
    std::cout << static_cast<bool>(x) << " ";
  }

  std::cout << std::endl;

  return 0;
}