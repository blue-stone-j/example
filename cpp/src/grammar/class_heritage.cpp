
// class_heritages
/*
Base class constructor is called.

Derived class member variables are initialized (in the order of declaration).

Derived class constructor body executes.
*/

#include <iostream>

class Base
{
 public:
  Base(int x) { std::cout << "Base constructor\n"; }
};

class Derived : public Base
{
 public:
  Derived(int x) :
    Base(x) { std::cout << "Derived constructor\n"; }
};

int main()
{
  Derived d(42);
}