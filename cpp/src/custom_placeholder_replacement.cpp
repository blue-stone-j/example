#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// converts any type with operator<< into std::string.
template <typename T>
std::string to_string_any(const T &value)
{
  std::ostringstream oss;
  oss << value;
  return oss.str();
}

// scans the format string and replaces {} sequentially with arguments.
template <typename... Args>
std::string simple_format(const std::string &fmt, Args &&... args)
{
  std::vector<std::string> values = {to_string_any(std::forward<Args>(args))...};

  std::string result;
  result.reserve(fmt.size() + values.size() * 8); // heuristic

  std::size_t arg_index = 0;
  for (std::size_t i = 0; i < fmt.size(); ++i)
  {
    if (fmt[i] == '{' && i + 1 < fmt.size() && fmt[i + 1] == '}')
    {
      if (arg_index < values.size())
      {
        result += values[arg_index++];
      }
      else
      {
        throw std::runtime_error("Not enough arguments for format string");
      }
      ++i; // skip '}'
    }
    else
    {
      result += fmt[i];
    }
  }

  if (arg_index < values.size())
  {
    throw std::runtime_error("Too many arguments for format string");
  }

  return result;
}

// Example usage
int main()
{
  std::string s = simple_format("Hello, {}! You have {} new messages.", "Alice", 5);
  std::cout << s << std::endl;
  return 0;
}
