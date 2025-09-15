/*
Unlike a pointer, std::optional actually owns its value (no dynamic allocation by default).
Works with most types, including custom structs/classes.
Has move and copy semantics.
*/
#include <optional>
#include <iostream>

std::optional<int> getValue(bool condition)
{
  if (condition)
  {
    return 42; // has value
  }
  else
  {
    return std::nullopt; // no value
  }
}

int main()
{
  auto result = getValue(true);
  if (result)
  {                                            // or result.has_value()
    std::cout << "Value: " << *result << "\n"; // dereference with *
  }
  else
  {
    std::cout << "No value\n";
  }

  std::optional<int> x = 10;
  std::cout << x.value_or(-1); // prints 10

  std::optional<int> y = std::nullopt;
  std::cout << y.value_or(-1); // prints -1

  return 0;
}
