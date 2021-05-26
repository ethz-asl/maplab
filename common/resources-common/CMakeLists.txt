cmake_minimum_required(VERSION 2.8.3)
project(resources_common)
find_package(catkin_simple REQUIRED)
catkin_simple(REQUIRED)
find_package(Draco REQUIRED)
#############
# LIBRARIES #
#############
link_directories(${draco_LIBRARY_DIR})
cs_add_library(libtinyply
  src/tinyply/tinyply.cc)
cs_add_library(${PROJECT_NAME}
  src/resources-gflags.cc
  src/point-cloud.cc)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} libdraco.so)
############
## EXPORT ##
############
cs_install()
cs_export()
