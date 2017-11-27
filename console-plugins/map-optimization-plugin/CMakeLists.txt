cmake_minimum_required (VERSION 2.8)
project(map_optimization_plugin)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-fPIC -shared)

cs_add_library(${PROJECT_NAME} src/optimizer-plugin.cc)
create_console_plugin(${PROJECT_NAME})

cs_install()
cs_export(INCLUDE_DIRS include ${CMAKE_CURRENT_BINARY_DIR})
