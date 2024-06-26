# 指定cmake的最低版本。在实际使用时，如果涉及的项目不复杂，随便选择一个版本即可。如果报错了再重选最低版本即可。
cmake_minimum_required(VERSION 3.10.2)

# 指定该项目的名称，此处该项目的名称即为“name”
project(name)

# 编译类型为release，不是debug
set(CMAKE_BUILD_TYPE "Release")

# 设置C++编译选项，使用C++17版本标准库；
set(CMAKE_CXX_FLAGS "-std=c++17")

# 设置编译类型为Release时的编译选项：-03（最高级优化） -wall（开启警告） -g（开启调试）
# pthread是linux下的线程库，用了多线程就要链接这个库，这时候要在编译选项上增加-pthread或者-lpthread
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

include(cmake/glog.cmake)

#
include_directories(
  include
  ${PROJECT_INCLUDE_DIR}
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
pkg_check_modules(YAML_CPP yaml-cpp)

find_package(Lua)

# ##########
# # Build ##
# ##########
# add_executable(cppio src/cppio.cpp)
# target_include_directories(cppio PRIVATE ${LUA_INCLUDE_DIR} ${JSONCPP_INCLUDE_DIRS} ${YAML_CPP_INCLUDEDIR})
# target_link_libraries(cppio ${YAML_CPP_LIBRARIES} ${JSONCPP_LIBRARIES} ${LUA_LIBRARIES})
add_executable(string_huff src/string_huff.cpp)

add_executable(multi_type src/multi_type.cpp)

add_executable(inverse_array src/inverse_array.cpp)

add_executable(union src/union.cpp)

add_executable(delete_element_array src/delete_element_array.cpp)

add_executable(cout src/cout.cpp)

add_executable(longest_string src/longest_string.cpp)

add_executable(atexit src/atexit.cpp)

add_executable(virtual_inherit src/virtual_inherit.cpp)

add_executable(cmatrix src/cmatrix.cpp)

add_executable(template_func src/template_func.cpp)

add_executable(typeid src/typeid.cpp)

add_executable(type_conversion src/type_conversion.cpp)

add_executable(template_class_inherit src/template_class_inherit.cpp)

add_executable(container src/container.cpp)

add_executable(global_varible src/global_varible.cpp)

add_executable(try_catch src/try_catch.cpp)

add_executable(matrix src/matrix.cpp)

add_executable(greatest_common_divisor src/greatest_common_divisor.cpp)

add_executable(prime_number src/prime_number.cpp)

add_executable(maze src/maze.cpp)

add_executable(bag01 src/dynamic_programming/bag/bag01.cpp)

add_executable(backtrack1 src/backtrack1.cpp)

add_executable(backtrack2 src/backtrack2.cpp)

add_executable(backtrack2 src/backtrack2.cpp)

# add_executable(main src/main.cpp)
