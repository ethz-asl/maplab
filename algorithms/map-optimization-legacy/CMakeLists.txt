cmake_minimum_required(VERSION 2.8.3)
project(map_optimization_legacy)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

SET(MAP_OPTIMIZATION_SOURCE src/ba-optimization-options.cc
                       src/double-window.cc
                       src/graph-ba-optimizer.cc
                       src/landmark-covariance-estimation.cc
                       src/landmark-geometry-verification.cc
                       src/pose-prior-import.cc
                       src/test/6dof-test-trajectory-gen.cc)
cs_add_library(${PROJECT_NAME} ${MAP_OPTIMIZATION_SOURCE})

SET(VIOPT_TEST_SOURCE src/test/6dof-test-trajectory-gen.cc
                      src/test/6dof-pose-graph-gen.cc
                      src/test/6dof-vi-map-gen.cc)

cs_add_library(${PROJECT_NAME}_test ${VIOPT_TEST_SOURCE})
target_link_libraries(${PROJECT_NAME}_test pthread)

file(COPY test/test-data DESTINATION
       ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_add_gtest(test_4dof_visual_ba_test
  test/test_4dof_visual_ba_test.cc)
target_link_libraries(test_4dof_visual_ba_test ${PROJECT_NAME})

catkin_add_gtest(test_6dof_trajectory_test
  test/test_6dof_trajectory_test.cc)
target_link_libraries(test_6dof_trajectory_test ${PROJECT_NAME})

catkin_add_gtest(test_6dof_ba_test
  test/test_6dof_ba_test.cc)
target_link_libraries(test_6dof_ba_test ${PROJECT_NAME}_test)

catkin_add_gtest(test_6dof_full_visual_ba_test
  test/test_6dof_full_visual_ba_test.cc)
target_link_libraries(test_6dof_full_visual_ba_test ${PROJECT_NAME}_test)

catkin_add_gtest(test_basic_viwls_graph_test
  test/test_basic_viwls_graph_test.cc)
target_link_libraries(test_basic_viwls_graph_test ${PROJECT_NAME})

catkin_add_gtest(test_landmark_association_test
  test/test_landmark_association_test.cc)
target_link_libraries(test_landmark_association_test ${PROJECT_NAME})

catkin_add_gtest(test_landmark_delete_test
  test/test_landmark_delete_test.cc)
target_link_libraries(test_landmark_delete_test ${PROJECT_NAME})

catkin_add_gtest(test_landmark_geometry_test
  test/test_landmark_geometry_test.cc)
target_link_libraries(test_landmark_geometry_test ${PROJECT_NAME})

catkin_add_gtest(test_landmark_merge_test
  test/test_landmark_merge_test.cc)
target_link_libraries(test_landmark_merge_test ${PROJECT_NAME})

catkin_add_gtest(test_mission_observer_count_test
  test/test_mission_observer_count_test.cc)
target_link_libraries(test_mission_observer_count_test ${PROJECT_NAME})

catkin_add_gtest(test_mission_duplicate_test
  test/test_mission_duplicate_test.cc)
target_link_libraries(test_mission_duplicate_test ${PROJECT_NAME})

catkin_add_gtest(test_optimizer_copy_test
  test/test_optimizer_copy_test.cc)
target_link_libraries(test_optimizer_copy_test ${PROJECT_NAME})

catkin_add_gtest(test_remove_mission_test
  test/test_remove_mission_test.cc)
target_link_libraries(test_remove_mission_test ${PROJECT_NAME}  ${PROJECT_NAME}_test)

catkin_add_gtest(test_remove_vertex_edge_test
  test/test_remove_vertex_edge_test.cc)
target_link_libraries(test_remove_vertex_edge_test ${PROJECT_NAME}  ${PROJECT_NAME}_test)

catkin_add_gtest(test_vertex_getter_test
  test/test_vertex_getter_test.cc)
target_link_libraries(test_vertex_getter_test ${PROJECT_NAME} ${PROJECT_NAME}_test)

catkin_add_gtest(test_visual_ba_test
  test/test_visual_ba_test.cc)
target_link_libraries(test_visual_ba_test ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
