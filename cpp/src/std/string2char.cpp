

#include <cstring>
#include <string>
#include <vector>

int main(int argc, char *argv[])

{
  std::string string = "Hello";
  const char *chars1 = string.c_str();
  delete[] chars1;

  std::vector<char> buffer(string.begin(), string.end());
  buffer.push_back('\0');                // Ensure null-termination
  char *modifiable_cstr = buffer.data(); // Safe, memory-managed, and avoids manual new/delete.

  char *chars2 = new char[string.size() + 1]; // +1 for null terminator
  std::strcpy(chars2, string.c_str());
  delete[] chars2; // free memory
}