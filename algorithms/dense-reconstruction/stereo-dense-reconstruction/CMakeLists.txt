cmake_minimum_required (VERSION 2.8)
project(stereo_dense_reconstruction)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(--std=c++11)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

cs_add_library(${PROJECT_NAME} src/stereo-camera-utils.cpp
                               src/disparity-conversion-utils.cpp
                               src/aslam-cv-interface.cpp
                               src/stereo-matcher.cpp
                               src/stereo-dense-reconstruction.cpp
                               src/stereo-pair-detection.cpp)

set(PROJECT_TEST_DATA "${PROJECT_NAME}_test_data")
execute_process(COMMAND rm -rf ${PROJECT_TEST_DATA}
                COMMAND tar -xvzf ${MAPLAB_TEST_DATA_DIR}/${PROJECT_TEST_DATA}/${PROJECT_TEST_DATA}.tar.gz -C .)

catkin_add_gtest(${PROJECT_NAME}_test test/test-stereo-dense-reconstruction.cpp)
target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})

cs_install()
cs_export()
