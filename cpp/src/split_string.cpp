/*
split string by delimiter
*/

#include <iostream>
#include <sstream>
#include <vector>
#include <string>


int main()
{
  { // Use std::stringstream if you want clarity and don’t care about tiny overhead.
    std::string s = "apple_banana_orange";
    std::stringstream ss(s);
    std::string part;
    std::vector<std::string> result;

    while (std::getline(ss, part, '_'))
    {
      result.push_back(part);
    }

    for (const auto &word : result)
    {
      std::cout << word << "\n";
    }
  }

  { // Use find + substr if you want full control and best portability.
    std::string s = "apple_banana_orange";
    std::vector<std::string> result;

    std::size_t start = 0, pos;
    while ((pos = s.find('_', start)) != std::string::npos)
    {
      result.push_back(s.substr(start, pos - start));
      start = pos + 1;
    }
    result.push_back(s.substr(start)); // last part

    for (const auto &word : result)
    {
      std::cout << word << "\n";
    }
  }

  // { // Use ranges::split if you’re on C++20 and like modern expressive style.
  //   std::string s = "apple_banana_orange";
  //   std::vector<std::string> result;

  //   for (auto &&part : std::views::split(s, '_'))
  //   {
  //     result.emplace_back(part.begin(), part.end());
  //   }

  //   for (const auto &word : result)
  //   {
  //     std::cout << word << "\n";
  //   }
  // }
}