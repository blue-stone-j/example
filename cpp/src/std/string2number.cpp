
#include <cstdlib>
#include <iostream>
#include <sstream>

int main()
{
  char *str           = (char *)"1234";
  int integer         = std::atoi(str); // Converts to int
  double double_float = std::atof(str); // Converts to double

  char *string_mixed = (char *)"1234abc";
  char *end;
  long long_integer = std::strtol(string_mixed, &end, 10); // base 10
  if (*end != '\0')
  {
    std::cout << "Warning: non-numeric characters found: " << end << std::endl;
  }

  // like strtol, strtod is used for double conversion
  double value_checked = std::strtod(string_mixed, &end);
  if (*end != '\0')
  {
    std::cout << "Warning: non-numeric characters found: " << end << std::endl;
  }

  char *string_complex = (char *)"1234.56";
  std::stringstream ss(string_complex);
  double d;
  ss >> d;
  if (ss.fail())
  {
    std::cout << "Conversion failed" << std::endl;
  }

  char *string_wrong = (char *)"abc";
  int value          = std::atoi(string_wrong); // result: 0

  return 0;
}