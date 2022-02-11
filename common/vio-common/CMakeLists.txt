cmake_minimum_required(VERSION 2.8.3)
project(vio_common)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############
if(APPLE)
  add_definitions(-DGTEST_HAS_TR1_TUPLE=0)
endif()
SET(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "${CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS} -lpthread")

cs_add_library(${PROJECT_NAME}
  src/imu-forward-propagation.cc
  src/imu-measurements-buffer.cc
  src/pose-lookup-buffer.cc
  src/rostopic-settings.cc
  src/test/vio-update-simulation.cc
)

##########
# GTESTS #
##########
catkin_add_gtest(test_imu_measurements_buffer test/test-imu-measurements-buffer.cc)
target_link_libraries(test_imu_measurements_buffer ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
