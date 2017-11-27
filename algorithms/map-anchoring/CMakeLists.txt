cmake_minimum_required(VERSION 2.8.3)
project(map_anchoring)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME} 
  src/map-anchoring.cc)

cs_install()
cs_export()
