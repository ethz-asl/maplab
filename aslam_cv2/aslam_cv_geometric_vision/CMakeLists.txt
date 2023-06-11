cmake_minimum_required(VERSION 2.8.3)
project(aslam_cv_geometric_vision)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############
set(HEADERS
  include/aslam/geometric-vision/match-outlier-rejection-twopt.h
  include/aslam/geometric-vision/pnp-pose-estimator.h
)

set(SOURCES
  src/match-outlier-rejection-twopt.cc
  src/pnp-pose-estimator.cc
)

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})
add_doxygen(NOT_AUTOMATIC)

add_definitions(-std=c++11)
SET(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "${CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS} -lpthread")

#########
# TESTS #
#########
catkin_add_gtest(test_pnp_pose_estimator_test
  test/test_pnp_pose_estimator_test.cc)
target_link_libraries(test_pnp_pose_estimator_test ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
