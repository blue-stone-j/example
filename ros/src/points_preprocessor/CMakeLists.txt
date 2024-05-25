cmake_minimum_required(VERSION 3.10)
project(points_preprocessor)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf2
)

catkin_package()

find_package(PCL REQUIRED)
find_package(OpenMP)

# Resolve system dependency on yaml-cpp, which apparently does not provide a CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR NAMES yaml_cpp.h PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY NAMES YAML_CPP PATHS ${YAML_CPP_LIBRARY_DIRS})
link_directories(${YAML_CPP_LIBRARY_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

link_directories(${PCL_LIBRARY_DIRS})

# Space Filter
add_executable(space_filter nodes/space_filter/space_filter.cpp)
target_link_libraries(space_filter ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(space_filter ${catkin_EXPORTED_TARGETS})

# Points Concat filter
add_executable(points_concat_filter nodes/points_concat_filter/points_concat_filter.cpp)
target_include_directories(points_concat_filter PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(points_concat_filter ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(points_concat_filter ${catkin_EXPORTED_TARGETS})

# Cloud Transformer
add_executable(cloud_transformer nodes/cloud_transformer/cloud_transformer_node.cpp)
target_include_directories(cloud_transformer PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(cloud_transformer ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(cloud_transformer ${catkin_EXPORTED_TARGETS})

# Compare Map Filter
add_executable(compare_map_filter nodes/compare_map_filter/compare_map_filter.cpp)
target_include_directories(compare_map_filter PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(compare_map_filter ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(compare_map_filter ${catkin_EXPORTED_TARGETS})

install(
  TARGETS
  cloud_transformer
  points_concat_filter
  space_filter
  compare_map_filter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
