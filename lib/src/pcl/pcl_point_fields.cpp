
#include <iostream>

#include <pcl/point_traits.h>

template <typename PointT>
void printPointTypeFields()
{
  std::cout << "Fields in the point type:\n";
  for (const auto &field : pcl::traits::fieldList<PointT>::type::fields)
  {
    std::cout << "  " << field.name << "\n";
  }
}

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    std::cerr << "Usage: " << argv[0] << " <bag_file> <cloud_topic>" << std::endl;
    return 1;
  }

  std::string cloud_file = argv[1];
}