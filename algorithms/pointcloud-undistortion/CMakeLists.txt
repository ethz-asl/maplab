cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_undistortion)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME} src/undistortion.cc)

############
## EXPORT ##
############
cs_install()
cs_export()
