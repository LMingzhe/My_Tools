cmake_minimum_required(VERSION 3.0)
project(exp_tool)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/bin)

find_package(PCL REQUIRED)

include_directories("/usr/local/include/eigen3")

# add_executable(binTpcd src/binTpcd.cpp)
# target_link_libraries(binTpcd ${PCL_LIBRARIES})

add_executable(double_filter src/double_filter.cpp)
target_link_libraries(double_filter ${PCL_LIBRARIES})

