cmake_minimum_required(VERSION 2.8.3)
project(lidar_feature_extraction)
add_compile_options(-w -fmax-errors=1)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3 -msse -msse2 -msse3 -msse4")

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}
  src/image-projection.cc)

cs_install()
cs_export()
