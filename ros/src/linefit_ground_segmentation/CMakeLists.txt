cmake_minimum_required(VERSION 3.10)

project(linefit_ground_segmentation)

set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -pthread")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include

  CATKIN_DEPENDS
  roscpp
  pcl
  std_msgs
  sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(ground_segmentation_node src/ground_segmentation_node.cc src/ground_segmentation.cpp)
add_dependencies(ground_segmentation_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(ground_segmentation_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} glog)

add_executable(ground_segmentation_test_node src/ground_segmentation_test_node.cc src/ground_segmentation.cpp)
add_dependencies(ground_segmentation_test_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(ground_segmentation_test_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} glog)
