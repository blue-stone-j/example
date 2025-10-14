/*
usage of std::variant
*/

#include <iostream>
#include <variant>

int main(int argc, char **argv)
{
  std::variant<int, float, std::string> v;

  v = 42; // holds an int
  std::cout << std::get<int>(v) << std::endl;

  v = 3.14f; // holds a float
  std::cout << std::get<float>(v) << std::endl;

  v = "Hello, World!"; // holds a string
  std::cout << std::get<std::string>(v) << std::endl;

  std::visit([](auto &&arg) { std::cout << "Visiting value: " << arg << std::endl; }, v);

  return 0;
}