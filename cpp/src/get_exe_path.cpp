
#include <iostream>
#include <string>

#if defined(_WIN32)
#include <windows.h>

std::string getExecutablePath()
{
  char buffer[MAX_PATH];
  DWORD length = GetModuleFileNameA(NULL, buffer, MAX_PATH); // GetModuleFileNameW for wide-character support if needed.
  if (length == 0 || length == MAX_PATH)
  {
    throw std::runtime_error("Failed to get executable path");
  }
  return std::string(buffer, length);
}

int main()
{
  try
  {
    std::string path = getExecutablePath();
    std::cout << "Executable path: " << path << std::endl;
  }
  catch (const std::exception &e)
  {
    std::cerr << "Error: " << e.what() << std::endl;
  }
  return 0;
}

#elif defined(__linux__)

#include <unistd.h>
#include <vector>

std::string getExecutablePath()
{
  std::vector<char> buffer(1024);
  ssize_t len = readlink("/proc/self/exe", buffer.data(), buffer.size() - 1);
  if (len == -1)
  {
    throw std::runtime_error("Failed to read /proc/self/exe");
  }
  buffer[len] = '\0';
  return std::string(buffer.data());
}

#endif

int main()
{
  try
  {
    std::string path = getExecutablePath();
    std::cout << "Executable path: " << path << std::endl;
  }
  catch (const std::exception &e)
  {
    std::cerr << "Error: " << e.what() << std::endl;
  }
  return 0;
}