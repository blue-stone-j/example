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
include(cmake/YAML.cmake)

#
include_directories(
  include
  ${PROJECT_INCLUDE_DIR}
  ${JSONCPP_INCLUDE_DIRS}
)
find_package(PkgConfig)
pkg_check_modules(JSONCPP jsoncpp)

find_package(Lua REQUIRED)

# ##########
# # Build ##
# ##########
add_executable(cppio src/cppio.cpp)
target_include_directories(cppio PRIVATE ${LUA_INCLUDE_DIR})
target_link_libraries(cppio
  ${YAML_CPP_LIBRARIES}
  ${JSONCPP_LIBRARIES}
  ${LUA_LIBRARIES}
)
