cmake_minimum_required(VERSION 2.8.3)
project(resources_common)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############
cs_add_library(libtinyply
               src/point-cloud.cc
               src/tinyply/tinyply.cc)


############
## EXPORT ##
############
cs_install()
cs_export()
