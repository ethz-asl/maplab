cmake_minimum_required(VERSION 2.8.3)
project(map_sparsification)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#########
# PROTO #
#########
set(PROTO_DEFNS proto/map-sparsification/keyframing-heuristics-options.proto)
PROTOBUF_CATKIN_GENERATE_CPP2("proto" PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME} ${PROTO_SRCS} ${PROTO_HDRS}
  src/graph-partition-sampler.cc
  src/optimization/lp-solve-sparsification.cc
  src/optimization/quadratic-term.cc
  src/keyframe-pruning.cc
  src/sampler-base.cc
  src/sampler-factory.cc
  src/heuristic/heuristic-sampling.cc
  src/heuristic/no-sampling.cc
  src/heuristic/random-sampling.cc
  src/visualization/map-sparsification-visualization.cc)

catkin_add_gtest(test_heuristic_landmark_sparsification
  test/test_heuristic_landmark_sparsification.cc)
target_link_libraries(test_heuristic_landmark_sparsification ${PROJECT_NAME})

catkin_add_gtest(test_lpsolve_api test/test_lpsolve_api.cc)
target_link_libraries(test_lpsolve_api ${PROJECT_NAME})

catkin_add_gtest(test_lpsolve_landmark_sparsification
  test/test_lpsolve_landmark_sparsification.cc)
target_link_libraries(test_lpsolve_landmark_sparsification ${PROJECT_NAME})
maplab_import_test_maps(test_lpsolve_landmark_sparsification)

cs_install()
cs_export()
