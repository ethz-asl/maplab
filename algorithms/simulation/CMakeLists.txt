cmake_minimum_required(VERSION 2.8.3)
project(simulation)

link_directories(${CATKIN_DEVEL_PREFIX}/lib)

add_definitions(--std=c++11)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(PROTO_DEFNS proto/path-4d.proto)
PROTOBUF_CATKIN_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

#############
# LIBRARIES #
#############

cs_add_library(${PROJECT_NAME}
  src/generic-path-generator.cc
  src/path_serialization.cc
  src/visual-inertial-path-generator.cc
  src/visual-nframe-simulator.cc
  ${PROTO_SRCS}
)

cs_add_executable(${PROJECT_NAME}_path src/generic-path-generator-demo.cc)
target_link_libraries(${PROJECT_NAME}_path ${PROJECT_NAME})

##########
# GTESTS #
##########
catkin_add_gtest(test_nframe_simulator test/test-nframe-simulator.cc)
target_link_libraries(test_nframe_simulator ${PROJECT_NAME})
add_custom_command(TARGET test_nframe_simulator
  COMMAND tar xvfz ${MAPLAB_TEST_DATA_DIR}/maplab_common_test_data/maplab_common_path_generation_samples.tar.gz)

##########
# EXPORT #
##########
cs_install()

set(SIMULATION_TEST_DATA "simulation_test_data")
set(SIMULATION_SHARE_DIR ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_SHARE_DESTINATION}/simulation)
execute_process(COMMAND rm -rf ${SIMULATION_SHARE_DIR}/test_paths
                COMMAND mkdir -p ${SIMULATION_SHARE_DIR}
                COMMAND tar -xvzf ${MAPLAB_TEST_DATA_DIR}/${SIMULATION_TEST_DATA}/${SIMULATION_TEST_DATA}.tar.gz -C ${SIMULATION_SHARE_DIR})
catkin_add_env_hooks(40.simulation
                     SHELLS sh
                     DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

cs_export()
