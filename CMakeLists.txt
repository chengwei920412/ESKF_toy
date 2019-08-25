cmake_minimum_required(VERSION 2.8.9)
project(ESKF_toy)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")

# for Eigen
set(CMAKE_MODULE_PATH cmake)
find_package(Eigen3)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  include
)

# build demo
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(ESKF_toy 
  src/dataParser.cpp
  src/ESKF.cpp
  src/main.cpp
  src/common_utils.cpp
)