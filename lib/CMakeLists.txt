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


### eigen ###
add_executable(compute_vector_rotation src/eigen/compute_vector_rotation.cpp)
target_link_libraries(compute_vector_rotation ${PCL_LIBRARIES})

add_executable(eigen_base src/eigen/eigen_base.cpp)
target_include_directories(eigen_base PUBLIC ${EIGEN3_INCLUDE_DIRS})

### pcl ###
add_executable(pcl_calculate_normal_using_octree src/pcl/pcl_calculate_normal_using_octree.cpp)
target_link_libraries(pcl_calculate_normal_using_octree ${PCL_LIBRARIES})

add_executable(pcl_filter_inplace src/pcl/pcl_filter_inplace.cpp)
target_link_libraries(pcl_filter_inplace ${PCL_LIBRARIES})

add_executable(pcl_find_nearest_using_octree src/pcl/pcl_find_nearest_using_octree.cpp)
target_link_libraries(pcl_find_nearest_using_octree ${PCL_LIBRARIES})

add_executable(pcl_pass_through src/pcl/pcl_pass_through.cpp)
target_link_libraries(pcl_pass_through ${PCL_LIBRARIES})

add_executable(pcl_point_fields src/pcl/pcl_point_fields.cpp)
target_link_libraries(pcl_point_fields ${PCL_LIBRARIES})

add_executable(pcl_region_growing src/pcl/pcl_region_growing.cpp)
target_link_libraries(pcl_region_growing ${PCL_LIBRARIES})

### ros ###
add_executable(ros_bag_extract_images src/ros/ros_bag_extract_images.cpp)
target_include_directories(ros_bag_extract_images PUBLIC ${rosbag_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS} ${cv_bridge_INCLUDE_DIRS})
target_link_libraries(ros_bag_extract_images ${rosbag_LIBRARIES} ${sensor_msgs_LIBRARIES} ${cv_bridge_LIBRARIES})

add_executable(ros_cloud_msg_fields src/ros/ros_cloud_msg_fields.cpp)
target_include_directories(ros_cloud_msg_fields PUBLIC ${rosbag_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS} ${pcl_INCLUDE_DIRS})
target_link_libraries(ros_cloud_msg_fields ${rosbag_LIBRARIES} ${sensor_msgs_LIBRARIES} ${pcl_LIBRARIES})

### gtsam ###
# add_executable(gtsam_usage src/gtsam_usage.cpp)
# target_link_libraries(gtsam_usage ${PCL_LIBRARIES})

# add_executable(gtsam_variant_camera src/gtsam/gtsam_variant_camera.cpp)
# target_link_libraries(gtsam_variant_camera PRIVATE gtsam)

add_executable(integrate_imu src/gtsam/integrate_imu.cpp)
target_link_libraries(integrate_imu gtsam)

add_executable(velocity_from_imu src/gtsam/velocity_from_imu.cpp)
target_link_libraries(velocity_from_imu gtsam)

### other ###
add_executable(exe src/main.cpp)
target_link_libraries(exe ${PCL_LIBRARIES})

# add_executable(g2o_usage src/g2o_usage.cpp)
# target_link_libraries(g2o_usage ${PCL_LIBRARIES})

add_executable(glog_usage src/glog_usage.cpp)
target_link_libraries(glog_usage PRIVATE glog)

add_executable(space src/space.cpp)
target_link_libraries(space ${PCL_LIBRARIES})

add_executable(sensor_extrinsic_viewer src/sensor_extrinsic_viewer.cpp)
target_include_directories(sensor_extrinsic_viewer PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(sensor_extrinsic_viewer PRIVATE ${PCL_LIBRARIES})
add_definitions(${PCL_DEFINITIONS})

add_executable(sophus_usage src/sophus_usage.cpp)
target_link_libraries(sophus_usage PRIVATE Sophus::Sophus)