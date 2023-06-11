cmake_minimum_required(VERSION 2.8.3)
project(aslam_cv_matcher)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############
set(HEADERS
  include/aslam/matcher/gyro-two-frame-matcher.h
  include/aslam/matcher/match.h
  include/aslam/matcher/match-helpers.h
)

set(SOURCES
  src/gyro-two-frame-matcher.cc
  src/match-helpers.cc
)

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

add_doxygen(NOT_AUTOMATIC)

SET(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "${CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS} -lpthread")

##########
# EXPORT #
##########
cs_install()
cs_export()
