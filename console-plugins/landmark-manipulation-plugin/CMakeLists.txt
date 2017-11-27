cmake_minimum_required (VERSION 2.8)
project(landmark_manipulation_plugin)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-fPIC -shared)

cs_add_library(${PROJECT_NAME} 
  src/landmark-manipulation-plugin.cc
)
create_console_plugin(${PROJECT_NAME})

cs_install()
cs_export()
