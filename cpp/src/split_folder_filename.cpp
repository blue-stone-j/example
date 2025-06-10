#include <iostream>
#include <filesystem>

int main()
{
  // split it into the directory part and the file name part,
  std::string path_str = "/home/user/data/file.txt";
  std::filesystem::path path(path_str);

  std::filesystem::path folder = path.parent_path();
  std::filesystem::path file   = path.filename();

  std::cout << "Directory: " << folder << std::endl;
  std::cout << "Filename : " << file << std::endl;


  // using string manipulation
  std::string full_path = "/home/user/data/file.txt";

  std::size_t found         = full_path.find_last_of("/\\");
  std::string folder_string = full_path.substr(0, found);
  std::string file_string   = full_path.substr(found + 1);

  std::cout << "Directory: " << folder_string << std::endl;
  std::cout << "Filename : " << file_string << std::endl;

  return 0;
}