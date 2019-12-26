cmake_minimum_required(VERSION 2.8.3)
project(maplab_ros_common)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})

cs_add_library(${PROJECT_NAME}  src/gflags-interface.cc)

cs_install()
cs_export()
