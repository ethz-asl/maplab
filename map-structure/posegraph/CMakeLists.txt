cmake_minimum_required(VERSION 2.8.3)
project(posegraph)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME} src/pose-graph.cc src/edge.cc src/vertex.cc)
target_link_libraries(${PROJECT_NAME} pthread)

cs_add_library(${PROJECT_NAME}_example_graph
  src/example/edge.cc
  src/example/pose-graph.cc
  src/example/vertex.cc)
target_link_libraries(${PROJECT_NAME}_example_graph
  ${PROJECT_NAME})

#############
## Testing ##
#############
catkin_add_gtest(test_posegraph test/test_posegraph.cc)
target_link_libraries(test_posegraph
  ${PROJECT_NAME}_example_graph)

catkin_add_gtest(test_posegraph_error_handling_test
  test/test_posegraph_error_handling_test.cc)
target_link_libraries(test_posegraph_error_handling_test
  ${PROJECT_NAME}_example_graph)

catkin_add_gtest(test_id_test test/test_id_test.cc)
target_link_libraries(test_id_test
  ${PROJECT_NAME}_example_graph)

##########
# EXPORT #
##########
cs_install()
cs_export()
