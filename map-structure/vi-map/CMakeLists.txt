cmake_minimum_required(VERSION 2.8.3)
project(vi_map)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(PROTO_DEFNS proto/vi-map/vi_map.proto  proto/vi-map/sensor_resources.proto)
PROTOBUF_CATKIN_GENERATE_CPP2("proto" PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})

SET(VI_MAP_SOURCE src/check-map-consistency.cc
                  src/cklam-edge.cc
                  src/edge.cc
                  src/landmark-quality-metrics.cc
                  src/landmark-store.cc
                  src/landmark.cc
                  src/loopclosure-edge.cc
                  src/mission-baseframe.cc
                  src/mission.cc
                  src/pose-graph.cc
                  src/semantic-landmark.cc
                  src/semantic-landmark-quality-metrics.cc
                  src/semantic-landmark-store.cc
                  src/sensor-manager.cc
                  src/sensor-utils.cc
                  src/test/vi-map-generator.cc
                  src/test/vi-map-test-helpers.cc
                  src/transformation-edge.cc
                  src/vertex.cc
                  src/vi-map-serialization.cc
                  src/vi-map.cc
                  src/vi-mission.cc
                  src/viwls-edge.cc)

cs_add_library(${PROJECT_NAME} ${VI_MAP_SOURCE} ${PROTO_SRCS})
target_link_libraries(${PROJECT_NAME})

##########
# GTESTS #
##########
catkin_add_gtest(test_edge_serialization
 test/test_edge_serialization.cc)
target_link_libraries(test_edge_serialization ${PROJECT_NAME})

catkin_add_gtest(test_edge_removal
 test/test_edge_removal.cc)
target_link_libraries(test_edge_removal ${PROJECT_NAME})

catkin_add_gtest(test_landmark
  test/test_landmark.cc)
target_link_libraries(test_landmark ${PROJECT_NAME})

catkin_add_gtest(test_map_consistency_check_test
test/test_map_consistency_check.cc)
target_link_libraries(test_map_consistency_check_test ${PROJECT_NAME})

catkin_add_gtest(test_semantic_landmark
  test/test_semantic_landmark.cc)
target_link_libraries(test_semantic_landmark ${PROJECT_NAME})

catkin_add_gtest(test_map_get_vertex_ids_in_mission_test
  test/test_map_get_vertex_ids_in_mission_test.cc)
target_link_libraries(test_map_get_vertex_ids_in_mission_test ${PROJECT_NAME})

catkin_add_gtest(test_remove_mission test/test-remove-mission.cc)
target_link_libraries(test_remove_mission ${PROJECT_NAME})

catkin_add_gtest(test_merge_map test/test_merge_map.cc)
target_link_libraries(test_merge_map ${PROJECT_NAME})

catkin_add_gtest(test_vi_mission_sensor_resources
test/test_vi_mission_sensor_resources.cc)
target_link_libraries(test_vi_mission_sensor_resources ${PROJECT_NAME})

catkin_add_gtest(test_vi_baseframe_protobuf_test
  test/test_vi_baseframe_protobuf_test.cc)
target_link_libraries(test_vi_baseframe_protobuf_test ${PROJECT_NAME})

catkin_add_gtest(test_vi_mission_protobuf_test
  test/test_vi_mission_protobuf_test.cc)
target_link_libraries(test_vi_mission_protobuf_test ${PROJECT_NAME})

# Keeping this for reference once we create a derived VIMap with summary maps.
# catkin_add_gtest(test_summary_landmark_accessors_test
#   test/test_summary_landmark_accessors_test.cc)
# target_link_libraries(test_summary_landmark_accessors_test ${PROJECT_NAME})

catkin_add_gtest(test_viwls_vertex_protobuf_test
  test/test_viwls_vertex_protobuf_test.cc)
target_link_libraries(test_viwls_vertex_protobuf_test ${PROJECT_NAME})

catkin_add_gtest(test_vertex_resources_test
  test/test_vertex_resources_test.cc)
target_link_libraries(test_vertex_resources_test ${PROJECT_NAME})

catkin_add_gtest(test_pmvs_resources_test
  test/test_pmvs_resources_test.cc)
target_link_libraries(test_pmvs_resources_test ${PROJECT_NAME})

file(COPY test/test_data DESTINATION
     ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})
set(TEST_WORKING_DIR  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_add_gtest(test_mission_selection test/test-mission-selection.cc)
target_link_libraries(test_mission_selection ${PROJECT_NAME})

catkin_add_gtest(test_serialization test/test-serialization.cc)
target_link_libraries(test_serialization ${PROJECT_NAME})

catkin_add_gtest(
    test_sensor_manager_serialization test/test-sensor-manager-serialization.cc)
target_link_libraries(test_sensor_manager_serialization ${PROJECT_NAME})

catkin_add_gtest(
    test_metadata_serialization test/test-metadata-serialization.cc)
target_link_libraries(test_metadata_serialization ${PROJECT_NAME})

catkin_add_gtest(
    test_edge_merging test/test_edge_merging.cc)
target_link_libraries(test_edge_merging ${PROJECT_NAME})

##############
# BENCHMARKS #
##############
add_benchmark(benchmark_map_serialization test/benchmark-vi-map-serialization.cc)
target_link_libraries(benchmark_map_serialization ${PROJECT_NAME})
maplab_import_test_maps(benchmark_map_serialization)

cs_install()
cs_export()
