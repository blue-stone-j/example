#include <map>

struct Inner
{
  explicit Inner(int){};
  // copyable and movable
};
struct OtherType
{
  Inner in;
};

int main(int argc, char **argv)
{
  {
    std::map<int, std::string> map;
    map[1] = "10";

    // get element
    auto &element     = map.at(1); // map.at(1) returns a reference
    auto element_copy = map.at(1); // map.at(1) returns a reference, copy is made from the int& into a new int, using the copy constructor.

    // add element
    // construct values in place.
    map.emplace(1, "hello");

    // with piecewise construction (avoids copying/moving complex value types)
    map.emplace(std::piecewise_construct,
                std::forward_as_tuple(2),
                std::forward_as_tuple("world"));

    // avoid unnecessary construction if the key already exists.
    map.try_emplace(3, "new value");    // inserts only if key=3 is absent
    map.try_emplace(1, "won't insert"); // key=1 already exists, nothing happens

    // This default-constructs the value if the key does not exist, then assigns to it.
    map[4] = "modified or inserted";

    int key = 1;
    std::string value("example");
    // require copying value when forming the pair (or when inserting), which fails if OtherType is move-only.
    map.insert(std::make_pair(key, std::move(value))); // OK if you let types deduce
  }

  { // look for elements
    std::map<int, std::string> map;
    map[1] = "10";

    // find: returns an iterator, or map.end() if not found
    auto it = map.find(1);
    if (it != map.end())
    {
      auto &element = it->second;
    }

    // at: an exception will be thrown if the key does not exist
    try
    {
      auto &element = map.at(1);
    }
    catch (const std::out_of_range &e)
    {
      // handle missing key
    }

    // []: constructs a default value if the key does not exist
    auto &element = map[1]; // element is a reference to the value at key 1
  }

  { // construct and insert complex types
    std::map<int, OtherType> map;

    int key = 1;
    OtherType value{Inner{10}};
    // or, without make_pair:
    // map.insert(std::pair<std::string, OtherType>(key, std::move(value)));
    // must construct the member explicitly
    map.emplace(key, OtherType{Inner{10}});
  }

  {
    std::map<int, std::string> map{{1, "apple"},
                                   {2, "banana"}};

    // Assume the map has exactly 2 elements
    auto it = map.begin();
    auto p  = std::make_pair(it->second, std::next(it)->second);
  }


  return 0;
}