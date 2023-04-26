cmake_minimum_required (VERSION 2.8)
project(grid_map_plugin)

#Again, not sure about this.
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#What is this?
add_definitions(-fPIC -shared)

cs_add_library(${PROJECT_NAME} src/grid-map-plugin.cc)
create_console_plugin(${PROJECT_NAME})

cs_install()
cs_export()
