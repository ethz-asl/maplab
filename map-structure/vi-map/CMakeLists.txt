cmake_minimum_required(VERSION 2.8.3)
project(vi_map)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(PROTO_DEFNS proto/vi-map/vi_map.proto  proto/vi-map/vi_map_deprecated.proto proto/vi-map/optional_camera_resources.proto)
PROTOBUF_CATKIN_GENERATE_CPP2("proto" PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})

add_definitions(--std=c++11)

SET(VI_MAP_SOURCE src/check-map-consistency.cc
                  src/cklam-edge.cc
                  src/edge.cc
                  src/gps-data-storage.cc
                  src/landmark.cc
                  src/landmark-quality-metrics.cc
                  src/landmark-store.cc
                  src/laser-edge.cc
                  src/loopclosure-edge.cc
                  src/mission.cc
                  src/mission-baseframe.cc
                  src/optional-sensor-data.cc
                  src/optional-sensor-extrinsics.cc
                  src/pose-graph.cc
                  src/sensor-manager.cc
                  src/sensor-manager-serialization.cc
                  src/semantics-manager.cc
                  src/structure-loopclosure-edge.cc
                  src/trajectory-edge.cc
                  src/transformation-edge.cc
                  src/vertex.cc
                  src/vi-map.cc
                  src/vi-map-serialization.cc
                  src/vi-map-serialization-deprecated.cc
                  src/vi-mission.cc
                  src/viwls-edge.cc
                  src/test/vi-map-generator.cc
                  src/test/vi-map-test-helpers.cc)

cs_add_library(${PROJECT_NAME} ${VI_MAP_SOURCE} ${PROTO_SRCS})
target_link_libraries(${PROJECT_NAME})

catkin_add_gtest(test_edge_serialization
 test/test_edge_serialization.cc)
target_link_libraries(test_edge_serialization ${PROJECT_NAME})

catkin_add_gtest(test_edge_removal
 test/test_edge_removal.cc)
target_link_libraries(test_edge_removal ${PROJECT_NAME})

catkin_add_gtest(test_landmark
  test/test_landmark.cc)
target_link_libraries(test_landmark ${PROJECT_NAME})

catkin_add_gtest(test_map_consistencycheck_test
  test/test_map_consistency_check.cc)
target_link_libraries(test_map_consistencycheck_test ${PROJECT_NAME})

catkin_add_gtest(test_map_get_vertex_ids_in_mission_test
  test/test_map_get_vertex_ids_in_mission_test.cc)
target_link_libraries(test_map_get_vertex_ids_in_mission_test ${PROJECT_NAME})

catkin_add_gtest(test_merge_map test/test_merge_map.cc)
target_link_libraries(test_merge_map ${PROJECT_NAME})

catkin_add_gtest(test_vi_mission_optional_camera_resources
  test/test_vi_mission_optional_camera_resources.cc)
target_link_libraries(test_vi_mission_optional_camera_resources ${PROJECT_NAME})

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

catkin_add_gtest(test_semantics_manager_test
  test/test_semantics_manager_test.cc)
target_link_libraries(test_semantics_manager_test ${PROJECT_NAME})

catkin_add_gtest(test_mission_selection test/test-mission-selection.cc)
target_link_libraries(test_mission_selection ${PROJECT_NAME})

catkin_add_gtest(test_serialization test/test-serialization.cc)
target_link_libraries(test_serialization ${PROJECT_NAME})

catkin_add_gtest(
    test_sensor_manager_serialization test/test-sensor-manager-serialization.cc)
target_link_libraries(test_sensor_manager_serialization ${PROJECT_NAME})

cs_install()
cs_export()
