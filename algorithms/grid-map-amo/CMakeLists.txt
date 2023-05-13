cmake_minimum_required(VERSION 2.8.3)
project(grid_map_amo)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############
add_definitions(-std=c++11)

cs_add_library(${PROJECT_NAME} 
  src/elevation-mapping.cc
  src/grid-map-filtering.cc
  src/orthomosaic.cc
  src/temperature-mapping.cc
  src/amo-combined.cc
)

#############
# QTCREATOR #
#############
FILE(GLOB_RECURSE LibFiles "include/*h")
add_custom_target(headers SOURCES ${LibFiles})

##########
# EXPORT #
##########
cs_install()
cs_export()
