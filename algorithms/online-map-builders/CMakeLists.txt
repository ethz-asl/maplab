cmake_minimum_required(VERSION 2.8.3)
project(online_map_builders)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -no-pie")

############
# LIBRARY  #
############
cs_add_library(
    ${PROJECT_NAME}
    src/stream-map-builder.cc)

##########
# EXPORT #
##########
cs_install()
cs_export()
