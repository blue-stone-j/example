# 指定cmake的最低版本。在实际使用时，如果涉及的项目不复杂，随便选择一个版本即可。如果报错了再重选最低版本即可。
cmake_minimum_required(VERSION 3.10.2)
# 指定该项目的名称，此处该项目的名称即为“name”
project(library_example)

# 编译类型为release，不是debug
set(CMAKE_BUILD_TYPE "Release")

# 设置C++编译选项，使用C++17版本标准库；
set(CMAKE_CXX_FLAGS "-std=c++17")

# 设置编译类型为Release时的编译选项：-03（最高级优化） -wall（开启警告） -g（开启调试）
# pthread是linux下的线程库，用了多线程就要链接这个库，这时候要在编译选项上增加-pthread或者-lpthread
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

include(cmake/eigen.cmake)
include(cmake/glog.cmake)
include(cmake/PCL.cmake)
include(cmake/OpenCV.cmake)
include(cmake/YAML.cmake)
include(cmake/global_definition.cmake)
include(cmake/GTSAM.cmake)

find_package(roscpp REQUIRED)
find_package(rosbag REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Sophus REQUIRED)
#
# include_directories(libmy lib3rd)
include_directories(${PROJECT_SOURCE_DIR})

# ##########
# # Build ##
# ##########

### pcl ###
add_executable(pcl_point_fields src/pcl/pcl_point_fields.cpp)
target_link_libraries(pcl_point_fields ${PCL_LIBRARIES})

add_executable(pcl_region_growing src/pcl/pcl_region_growing.cpp)
target_link_libraries(pcl_region_growing ${PCL_LIBRARIES})
add_executable(exe src/main.cpp)
target_link_libraries(exe ${PCL_LIBRARIES})

add_executable(space src/space.cpp)
target_link_libraries(space ${PCL_LIBRARIES})

add_executable(sensor_extrinsic_viewer src/sensor_extrinsic_viewer.cpp)
target_include_directories(sensor_extrinsic_viewer PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(sensor_extrinsic_viewer PRIVATE ${PCL_LIBRARIES})
add_definitions(${PCL_DEFINITIONS})
