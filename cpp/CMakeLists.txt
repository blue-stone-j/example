# 指定cmake的最低版本。在实际使用时，如果涉及的项目不复杂，随便选择一个版本即可。如果报错了再重选最低版本即可。
cmake_minimum_required(VERSION 3.16.2)

# 指定该项目的名称，此处该项目的名称即为"cpp_example"
project(cpp_example)

# 编译类型为release，不是debug
set(CMAKE_BUILD_TYPE "Release")

# 设置C++编译选项，使用C++17版本标准库
set(CMAKE_CXX_FLAGS "-std=c++17")

# 设置编译类型为Release时的编译选项：-03（最高级优化） -wall（开启警告） -g（开启调试）
# pthread是linux下的线程库，用了多线程就要链接这个库，这时候要在编译选项上增加-pthread或者-lpthread
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

include(cmake/glog.cmake)

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
pkg_check_modules(YAML_CPP yaml-cpp)

find_package(Lua REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)

# ##########
# # Build ##
# ##########

function(generate_executable_name FILENAME OUTPUT_VAR)
    string(REPLACE ".cpp" "" EXECUTABLE_NAME "${FILENAME}")
    string(REPLACE "/" "_" EXECUTABLE_NAME "${EXECUTABLE_NAME}")
    set(${OUTPUT_VAR} ${EXECUTABLE_NAME} PARENT_SCOPE)
endfunction()

link_libraries(${YAML_CPP_LIBRARIES} ${JSONCPP_LIBRARIES} ${LUA_LIBRARIES} ${EIGEN3_LIBRARIES} OpenMP::OpenMP_CXX)
include_directories(include  ${PROJECT_INCLUDE_DIR} ${LUA_INCLUDE_DIR} ${JSONCPP_INCLUDE_DIRS} ${YAML_CPP_INCLUDEDIR} ${EIGEN3_INCLUDE_DIR})

# Define a function to get all source files and create executables
function(create_executables)
  # Get all .cpp files
  file(GLOB_RECURSE CPP_FILES "${CMAKE_SOURCE_DIR}/*.cpp")

  # Iterate through each .cpp file
  foreach(CPP_FILE ${CPP_FILES})
    # Get the relative path and filename without extension
    file(RELATIVE_PATH RELATIVE_PATH ${CMAKE_SOURCE_DIR} ${CPP_FILE})
    get_filename_component(FILENAME_WE ${CPP_FILE} NAME_WE)

    # Get the directory part of the relative path
    get_filename_component(DIRNAME ${RELATIVE_PATH} DIRECTORY)

    # Check if DIRNAME is empty
    if(DIRNAME STREQUAL "")
      set(DIRNAME_SAFE "")
    else()
      # Replace '/' or '\' in the directory name with '_'
      string(REPLACE "/" "_" DIRNAME_SAFE ${DIRNAME})
      string(REPLACE "\\" "_" DIRNAME_SAFE ${DIRNAME_SAFE})
    endif()

    # Form the executable name
    if(DIRNAME_SAFE STREQUAL "")
      set(EXECUTABLE_NAME ${FILENAME_WE})
    else()
      set(EXECUTABLE_NAME "${DIRNAME_SAFE}_${FILENAME_WE}")
    endif()

    # Add executable target
    add_executable(${EXECUTABLE_NAME} ${CPP_FILE})
    # target_link_libraries(${EXECUTABLE_NAME} ${PCL_LIBRARIES})
  endforeach()
endfunction()

# Call the function to create executables
create_executables()