/**/

int main(int argc, char **argv)
{
  std::shared_ptr<int> p = std::make_shared<int>(42);

  // dereference the shared pointer
  std::cout << *p << std::endl; // prints 42

  int *raw = p.get();
  std::cout << *raw << std::endl; // prints 42

  // operator bool() Check Before Access
  if (p)
  {
    std::cout << *p << std::endl;
  }
  else
  {
    std::cout << "Pointer is null" << std::endl;
  }

  return 0;
}