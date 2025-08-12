#include <iostream>
#include <filesystem>

int main()
{
  // split it into the directory part and the file name part,
  std::string path_str = "/home/user/data/file.txt";
  std::filesystem::path path(path_str);

  std::filesystem::path folder = path.parent_path();
  std::filesystem::path file   = path.filename();  // with extension
  std::filesystem::path stem   = file.stem();      // without extension
  std::filesystem::path ext    = file.extension(); // just the extension

  std::cout << "Directory: " << folder << std::endl;
  std::cout << "Filename : " << file << std::endl;
  std::cout << "Stem     : " << stem << std::endl;
  std::cout << "Extension: " << ext << std::endl;

  // convert path to string using the platform’s preferred directory separators
  std::string path_string1 = path.string();
  // convert path to generic string using forward slashes (/), regardless of platform.
  std::string path_string2 = path.generic_string();

  // using string manipulation
  std::string full_path = "/home/user/data/file.txt";

  std::size_t found         = full_path.find_last_of("/\\");
  std::string folder_string = full_path.substr(0, found);
  std::string file_string   = full_path.substr(found + 1);

  std::cout << "Directory: " << folder_string << std::endl;
  std::cout << "Filename : " << file_string << std::endl;

  return 0;
}