cmake_minimum_required(VERSION 2.8)
project(inverted_multi_index)

find_package(catkin_simple REQUIRED)
catkin_simple()

set(LIBRARY_NAME ${PROJECT_NAME})

set(PROTO_DEFNS proto/inverted-multi-index/inverted_multi_index.proto)
PROTOBUF_CATKIN_GENERATE_CPP2("proto" PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})

cs_add_library(${LIBRARY_NAME} src/inverted-multi-index.cc
                               src/inverted-multi-product-quantization-index.cc
                               ${PROTO_SRCS})

catkin_add_gtest(test_inverted_multi_index test/test_inverted-multi-index.cc
                 WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
target_link_libraries(test_inverted_multi_index ${LIBRARY_NAME})

# TODO(magehrig): Re-enable these tests after fixing unsupported gmock checks.
#catkin_add_gmock(test_inverted_multi_index_common
#                 test/test_inverted-multi-index-common.cc
#                 WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/)
#target_link_libraries(test_inverted_multi_index_common ${catkin_LIBRARIES})
#catkin_add_gmock(test_inverted_multi_product_quantization_index
#                 test/test_inverted-multi-product-quantization-index.cc
#                 WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/)
#target_link_libraries(test_inverted_multi_product_quantization_index
#                      ${catkin_LIBRARIES})

cs_install()
cs_export()
