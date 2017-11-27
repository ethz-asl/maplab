cmake_minimum_required(VERSION 2.8.3)
project(online_map_builders)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

############
# LIBRARY  #
############
add_definitions(-std=c++11)

cs_add_library(
    ${PROJECT_NAME}
    src/keyframed-map-builder.cc
    src/stream-map-builder.cc)

##########
# EXPORT #
##########
cs_install()
cs_export()
