
/*
In C++ you cannot directly make int → enum conversions implicit (the language disallows it for safety reasons, because not all integers correspond to valid enumerators). By default:

enum → int is implicit.

int → enum is explicit (requires a cast).
*/

int main(int argc, char **argv)
{
  enum class Color
  {
    Red   = 1,
    Green = 2,
    Blue  = 3
  };

  int x   = 2;
  Color c = static_cast<Color>(x); // explicit cast required
}