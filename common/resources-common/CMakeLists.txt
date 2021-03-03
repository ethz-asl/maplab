cmake_minimum_required(VERSION 2.8.3)
project(resources_common)

find_package(catkin_simple)
catkin_simple(REQUIRED)
find_package(Draco REQUIRED)
#############
# LIBRARIES #
#############
catkin_package(
  INCLUDE_DIRS include ${draco_INCLUDE_DIR}
  LIBRARIES ${PROJECT_NAME} ${draco_LIBRARY_DIR}
)
include_directories(include
        ${catkin_INCLUDE_DIRS}
        ${draco_INCLUDE_DIR}/..
        ${draco_INCLUDE_DIR})

link_directories(${draco_LIBRARY_DIR})

cs_add_library(libtinyply
  src/tinyply/tinyply.cc)
cs_add_library(${PROJECT_NAME}
  src/resources-gflags.cc)
############
## EXPORT ##
############
cs_install()
cs_export()
