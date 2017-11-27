cmake_minimum_required(VERSION 2.8.3)
project(vio_common)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

############
# PROTOBUF #
############
SET(PROTO_DEFNS proto/vio-common/vio_update.proto
                proto/vio-common/vio_types.proto)
PROTOBUF_CATKIN_GENERATE_CPP2("proto" PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})

#############
# LIBRARIES #
#############
add_definitions(-std=c++11)

if(APPLE)
  add_definitions(-DGTEST_HAS_TR1_TUPLE=0)
endif()
SET(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "${CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS} -lpthread")

cs_add_library(${PROJECT_NAME} ${PROTO_SRCS} ${PROTO_HDRS}
  src/imu-measurements-buffer.cc
  src/test/vio-update-simulation.cc
  src/rostopic-settings.cc
  src/vio-update-serialization.cc
)

##########
# GTESTS #
##########
catkin_add_gtest(test_imu_measurements_buffer test/test-imu-measurements-buffer.cc)
target_link_libraries(test_imu_measurements_buffer ${PROJECT_NAME})

catkin_add_gtest(test_vio_update_serialization_test test/test-vio-update-serialization.cc)
target_link_libraries(test_vio_update_serialization_test ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
