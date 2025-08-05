
#include <iostream>

template <typename PointT>
void printPointTypeFields()
{
  std::cout << "Fields in the point type:\n";
  for (const auto &field : pcl::traits::fieldList<PointT>::type::fields)
  {
    std::cout << "  " << field.name << "\n";
  }
}