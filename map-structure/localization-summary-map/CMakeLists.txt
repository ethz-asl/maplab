cmake_minimum_required(VERSION 2.8.3)
project(localization_summary_map)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(PROTO_DEFNS proto/localization-summary-map/localization-summary-map.proto)
PROTOBUF_CATKIN_GENERATE_CPP2("proto" PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})

SET(LOCALIZATION_SUMMARY_MAP_SOURCE src/localization-summary-map.cc
                                    src/localization-summary-map-creation.cc)
cs_add_library(${PROJECT_NAME} ${LOCALIZATION_SUMMARY_MAP_SOURCE} ${PROTO_SRCS})

catkin_add_gtest(test_localization_summary_map_protobuf_test
                 test/test_localization_summary_map_protobuf_test.cc)
target_link_libraries(test_localization_summary_map_protobuf_test ${PROJECT_NAME})

catkin_add_gtest(test_localization_summary_map_test
                 test/test_localization_summary_map_test.cc)
target_link_libraries(test_localization_summary_map_test ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
