cmake_minimum_required(VERSION 3.1.3)
project(pointmatcher_ros)

# Set compiler flags
set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Wall -Wextra -Wpedantic)

# Set catkin package dependencies
set(CATKIN_PACKAGE_DEPENDENCIES
  eigen_conversions
  nav_msgs
  pcl_conversions
  roscpp
  sensor_msgs
  tf
  tf_conversions
)

# Find catkin macros and libraries
find_package(catkin
  REQUIRED COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(libpointmatcher REQUIRED)
find_package(PCL 1.7 REQUIRED)
add_definitions(${PCL_DEFINITIONS})

# Catkin package macro
catkin_package(
  INCLUDE_DIRS 
    include
    ${PCL_INCLUDE_DIRS}
  LIBRARIES
    pointmatcher_ros
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

add_definitions(-DHAVE_YAML_CPP)
include_directories(
  include
  SYSTEM
    ${LIBPOINTMATCHER_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

########################
## Library definition ##
########################
add_library(pointmatcher_ros
  src/point_cloud.cpp
  src/transform.cpp
  src/StampedPointCloud.cpp
  src/PmTf.cpp
  src/PointMatcherFilterInterface.cpp
  src/helper_functions.cpp
)

target_link_libraries(pointmatcher_ros
  ${catkin_LIBRARIES}
  ${LIBPOINTMATCHER_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
)

#############
## Install ##
#############
install(TARGETS pointmatcher_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
