cmake_minimum_required(VERSION 2.8)
project(descriptor_projection)

find_package(catkin_simple REQUIRED)
catkin_simple()

set(PROTO_DEFNS proto/descriptor-projection/projected_image.proto)
PROTOBUF_CATKIN_GENERATE_CPP2("proto" PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(LIBRARY_NAME ${PROJECT_NAME})

cs_add_library(${LIBRARY_NAME} src/build-projection-matrix.cc
                               src/descriptor-projection.cc
                               src/flags.cc
                               src/map-track-extractor.cc
                               src/projected-descriptor-quantizer.cc
                               src/train-projection-matrix.cc
                               ${PROTO_SRCS})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_descriptor_projection_build_projection_matrix test/test_build-projection-matrix.cc
                   WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/)
  target_link_libraries(test_descriptor_projection_build_projection_matrix ${LIBRARY_NAME})

  catkin_add_gtest(test_matching_based_lc_quantizer_serialization test/test_quantizer-serialization.cc
                   WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/)
  target_link_libraries(test_matching_based_lc_quantizer_serialization ${LIBRARY_NAME})
endif()

# CMake Indexing
FILE(GLOB_RECURSE LibFiles "include/*")
add_custom_target(headers SOURCES ${LibFiles})
FILE(GLOB_RECURSE ProtoFiles "proto/*")
add_custom_target(protos SOURCES ${ProtoFiles})

cs_install()
cs_export()
