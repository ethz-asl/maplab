cmake_minimum_required(VERSION 2.8.3)
project(aslam_cv_tracker)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############
set(HEADERS
  include/aslam/tracker/feature-tracker.h
  include/aslam/tracker/feature-tracker-gyro.h
  include/aslam/tracker/track-manager.h
)

set(SOURCES
  src/feature-tracker-gyro.cc
  src/track-manager.cc
  src/tracking-helpers.cc
)

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

add_doxygen(NOT_AUTOMATIC)

SET(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "${CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS} -lpthread")

##########
# GTESTS #
##########
catkin_add_gtest(test_track_manager test/test-track-manager.cc)
target_link_libraries(test_track_manager ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
