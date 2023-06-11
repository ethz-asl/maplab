cmake_minimum_required(VERSION 2.8.3)
project(aslam_cv_triangulation)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############
set(HEADERS
  include/aslam/triangulation/triangulation.h
)

set(SOURCES
  src/triangulation.cc
)

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# TODO(tcies) reenable once sm timing fixed in tango
# cs_add_executable(triangulation-benchmark benchmark/triangulation-benchmark.cc)
# target_link_libraries(triangulation-benchmark ${PROJECT_NAME} gtest pthread)

add_doxygen(NOT_AUTOMATIC)

SET(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "${CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS} -lpthread")

##########
# GTESTS #
##########
catkin_add_gtest(test_triangulation
  test/test-triangulation.cc
  include/aslam/triangulation/test/triangulation-fixture.h
)
target_link_libraries(test_triangulation ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
