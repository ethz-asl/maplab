cmake_minimum_required(VERSION 2.8.3)
project(aslam_cv_common)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/export_flags.cmake)

############
# PROTOBUF #
############
set(PROTO_DEFNS proto/aslam/common/id.proto)
PROTOBUF_CATKIN_GENERATE_CPP2("proto" PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

#############
# LIBRARIES #
#############
set(SOURCES
  src/channel.cc
  src/channel-serialization.cc
  src/covariance-helpers.cc
  src/hash-id.cc
  src/reader-first-reader-writer-lock.cc
  src/reader-writer-lock.cc
  src/sensor.cc
  src/statistics.cc
  src/thread-pool.cc
  src/timer.cc
  src/internal/unique-id.cc
  ${PROTO_SRCS}
  ${PROTO_HDRS}
)

cs_add_library(${PROJECT_NAME} ${SOURCES})

add_doxygen(NOT_AUTOMATIC)

##########
# GTESTS #
##########
catkin_add_gtest(test_channel-serialization test/test-channel-serialization.cc)
target_link_libraries(test_channel-serialization ${PROJECT_NAME})

catkin_add_gtest(test_channels test/test-channels.cc)
target_link_libraries(test_channels ${PROJECT_NAME})

catkin_add_gtest(test_eigen-yaml-serialization
  test/test-eigen-yaml-serialization.cc
)
target_link_libraries(test_eigen-yaml-serialization ${PROJECT_NAME})

catkin_add_gtest(test_hash_id test/test-hash-id.cc)
target_link_libraries(test_hash_id ${PROJECT_NAME})

catkin_add_gtest(test_stl_helpers test/test-stl-helpers.cc)
target_link_libraries(test_stl_helpers ${PROJECT_NAME})

catkin_add_gtest(test_covariance_helpers test/test-covariance-helpers.cc)
target_link_libraries(test_covariance_helpers ${PROJECT_NAME})

catkin_add_gtest(test_thread-pool test/test-thread-pool.cc)
target_link_libraries(test_thread-pool ${PROJECT_NAME})

catkin_add_gtest(test_time test/test-time.cc)
target_link_libraries(test_time ${PROJECT_NAME})

catkin_add_gtest(test_reader_writer_lock_test test/reader_writer_lock_test.cc)
target_link_libraries(test_reader_writer_lock_test ${PROJECT_NAME})
target_link_libraries(test_reader_writer_lock_test -pthread)

catkin_add_gtest(test_occupancy_grid test/test-occupancy-grid.cc)
target_link_libraries(test_occupancy_grid ${catkin_LIBRARIES})

catkin_add_gtest(test_descriptor_utils test/test-descriptor-utils.cc)
target_link_libraries(test_descriptor_utils ${catkin_LIBRARIES})


##########
# EXPORT #
##########
cs_install()
cs_export(CFG_EXTRAS detect_simd.cmake export_flags.cmake setup_openmp.cmake)
