
#include <filesystem>

int main(int argc, char **argv)
{
  std::filesystem::path p = "/tmp";
  if (std::filesystem::exists(p))
  {
    std::cout << p << " exists\n";
  }

  const std::filesystem::path filename = "test.txt";
  try
  {
    if (std::filesystem::remove(filename))
    {
      std::cout << "File deleted successfully.\n";
    }
    else
    {
      std::cout << "File does not exist.\n";
    }
  }
  catch (const std::filesystem::filesystem_error &e)
  {
    std::cerr << "Error: " << e.what() << '\n';
  }

  const std::filesystem::path dir = "my_directory";
  try
  {
    // Deletes only files, leaves subdirectories untouched.
    for (const auto &entry : std::filesystem::directory_iterator(dir))
    {
      if (std::filesystem::is_regular_file(entry.path()))
      {
        std::filesystem::remove(entry.path()); // delete only files
      }
    }
    std::cout << "All files removed successfully.\n";
  }
  catch (const std::filesystem::filesystem_error &e)
  {
    std::cerr << "Error: " << e.what() << '\n';
  }
  try
  {
    // removes file or directory recursively
    for (const auto &entry : std::filesystem::directory_iterator(dir))
    {
      std::filesystem::remove_all(entry.path());
    }
    std::cout << "All files removed successfully.\n";
  }
  catch (const std::filesystem::filesystem_error &e)
  {
    std::cerr << "Error: " << e.what() << '\n';
  }


  const fs::path source      = "my_directory/file.txt";
  const fs::path destination = "backup_folder/file.txt";
  try
  {
    fs::create_directories(destination.parent_path()); // ensure target folder exists
    fs::copy_file(source, destination, fs::copy_options::overwrite_existing);
    std::cout << "File copied successfully.\n";
  }
  catch (const fs::filesystem_error &e)
  {
    std::cerr << "Error: " << e.what() << '\n';
  }

  try
  {
    fs::create_directories(destination); // ensure destination exists

    fs::copy(source, destination, fs::copy_options::recursive | fs::copy_options::overwrite_existing);

    std::cout << "Folder copied successfully.\n";
  }
  catch (const fs::filesystem_error &e)
  {
    std::cerr << "Error: " << e.what() << '\n';
  }


  return 0;
}