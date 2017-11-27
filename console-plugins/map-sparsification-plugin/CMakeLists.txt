cmake_minimum_required(VERSION 2.8.3)
project(map_sparsification_plugin)

find_package(catkin_simple REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-fPIC -shared)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME} 
  src/keyframe-pruning.cc
  src/landmark-sparsification.cc
  src/map-sparsification-plugin.cc
)
create_console_plugin(${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
