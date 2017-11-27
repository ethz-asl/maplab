cmake_minimum_required(VERSION 2.8.3)
project(dense_reconstruction_common)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME} src/conversion-tools.cc
                               src/resource-utils.cc)

############
## EXPORT ##
############
cs_install()
cs_export()
