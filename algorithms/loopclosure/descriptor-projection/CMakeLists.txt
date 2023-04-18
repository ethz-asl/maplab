cmake_minimum_required(VERSION 2.8)
project(descriptor_projection)

find_package(catkin_simple REQUIRED)
catkin_simple()

include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(LIBRARY_NAME ${PROJECT_NAME})

cs_add_library(${LIBRARY_NAME} src/build-projection-matrix.cc
                               src/descriptor-projection.cc
                               src/flags.cc
                               src/map-track-extractor.cc
                               src/train-projection-matrix.cc)

catkin_add_gtest(test_descriptor_projection_build_projection_matrix test/test_build-projection-matrix.cc
                 WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/)
target_link_libraries(test_descriptor_projection_build_projection_matrix ${LIBRARY_NAME})

cs_install()
cs_export()
