cmake_minimum_required(VERSION 2.8.3)
project(sparse_graph)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
if(COMPILER_SUPPORTS_CXX14)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_FLAGS         "-Wall -Wextra -msse -msse2 -msse3 -msse4 -O3 -DNDEBUG")
  set(CMAKE_CXX_FLAGS_DEBUG   "-O0 -g")
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
else()
  message(FATAL_ERROR "The compiler does not support C++14.")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############

cs_add_library(${PROJECT_NAME}
  src/sparse-graph.cc
  src/mission-graph.cc
  src/dense-map-builder.cc
  src/partitioners/base-partitioner.cc
  src/partitioners/avg-partitioner.cc
  src/partitioners/all-partitioner.cc
  src/partitioners/lidar-partitioner.cc
  src/common/representative-node.cc
  src/common/utils.cc
  src/common/sparse-graph-gflags.cc
)

##########
# TESTS #
##########
catkin_add_gtest(test_sparse_graph test/test-sparse-graph.cc)
target_link_libraries(test_sparse_graph ${PROJECT_NAME})
catkin_add_gtest(test_partitioners test/test-partitioners.cc)
target_link_libraries(test_partitioners ${PROJECT_NAME})
catkin_add_gtest(test_representative_nodes test/test-representative-node.cc)
target_link_libraries(test_representative_nodes ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
