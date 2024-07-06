#include <iostream>

void decrease(void *data, int psize)
{
  switch (psize)
  {
    case 1:
      char *pchar;
      pchar = (char *)data;
      --(*pchar);
      break;
    case sizeof(int):
      int *pint;
      pint = (int *)data;
      --(*pint);
  }
}

int main()
{
  char a = 'd';
  int b  = 102;
  decrease(&a, sizeof(a));
  decrease(&b, sizeof(b));
  std::cout << a << "," << b << std::endl;

  return 0;
}