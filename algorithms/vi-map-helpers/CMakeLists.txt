cmake_minimum_required(VERSION 2.8.3)
project(vi_map_helpers)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME} 
  src/mission-clustering-coobservation.cc
  src/near-camera-pose-sampling.cc
  src/spatial-database-vertex-id.cc
  src/vi-map-descriptor-utils.cc
  src/vi-map-geometry.cc
  src/vi-map-landmark-quality-evaluation.cc
  src/vi-map-manipulation.cc
  src/vi-map-nearest-neighbor-lookup.cc
  src/vi-map-partitioner.cc
  src/vi-map-queries.cc
  src/vi-map-stats.cc
  src/vi-map-vertex-time-queries.cc
)

catkin_add_gtest(test_map_geometry_test
  test/test_map_geometry_test.cc)
target_link_libraries(test_map_geometry_test ${PROJECT_NAME})

catkin_add_gtest(test_mission_clustering_coobservation
  test/test_mission_clustering_coobservation.cc)
target_link_libraries(test_mission_clustering_coobservation ${PROJECT_NAME})

catkin_add_gtest(test_mission_observer_clustering
  test/test_mission_observer_clustering.cc)
target_link_libraries(test_mission_observer_clustering ${PROJECT_NAME})

catkin_add_gtest(test_queries_vertex_getter
  test/test_queries_vertex_getter.cc)
target_link_libraries(test_queries_vertex_getter ${PROJECT_NAME})

catkin_add_gtest(test_spatial_database_test
  test/test_spatial_database_test.cc)
target_link_libraries(test_spatial_database_test ${PROJECT_NAME})

catkin_add_gtest(test_nearest_neighbor_lookup_test
  test/test_nearest_neighbor_lookup_test.cc
)
target_link_libraries(test_nearest_neighbor_lookup_test ${PROJECT_NAME})

catkin_add_gtest(test_vertex_time_queries_test
  test/test_vertex_time_queries_test.cc
)
target_link_libraries(test_vertex_time_queries_test ${PROJECT_NAME})

cs_install()
cs_export()
