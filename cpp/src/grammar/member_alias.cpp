
#include <functional>

class Warpper
{
 public:
  int a;
  int b;
  std::reference_wrapper<int> alias;

  Warpper() :
    a(0), b(0), alias(a) {}
};


class ReferenceClass
{
  // Reference members must be initialized in the constructor initializer list.
  // They cannot be reseated (i.e., you can’t later make ref refer to another member).
  // Order of initialization follows declaration order, not the initializer list order.
 public:
  int value;
  int &ref; // reference member

  ReferenceClass(int v) :
    value(v), ref(value) {} // must be initialized in constructor
};

class PointerClass
{
 public:
  int a;
  int b;
  int *alias; // pointer member

  PointerClass() :
    a(0), b(0), alias(&a) {} // start aliasing a
};

int main(int argc, char **argv)
{
  {
    Warpper obj;
    obj.alias.get() = 5;     // modifies a
    obj.alias       = obj.b; // now alias refers to b
  }

  {
    ReferenceClass obj(42);
    obj.ref = 100; // modifies obj.value as well
  }

  {
    PointerClass obj;
    *(obj.alias) = 10;     // changes obj.a
    obj.alias    = &obj.b; // now alias refers to b
  }

  return 0;
}