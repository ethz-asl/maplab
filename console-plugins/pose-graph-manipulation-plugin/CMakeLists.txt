cmake_minimum_required (VERSION 2.8)
project(pose_graph_manipulation_plugin)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-fPIC -shared)

cs_add_library(${PROJECT_NAME} src/edge-manipulation.cc
                               src/pose-graph-manipulation-plugin.cc
                               src/reset-wheel-odometry.cc)
create_console_plugin(${PROJECT_NAME})

##################
# CMake Indexing #
##################
FILE(GLOB_RECURSE LibFiles "include/*")
add_custom_target(headers SOURCES ${LibFiles})

cs_install()
cs_export()
