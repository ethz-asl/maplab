cmake_minimum_required (VERSION 2.8)
project(vi_map_visualization_plugin)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-fPIC -shared)

cs_add_library(${PROJECT_NAME} src/visualization-plugin.cc)
create_console_plugin(${PROJECT_NAME})

cs_install()
cs_export()
