cmake_minimum_required(VERSION 2.8)
project(vocabulary_tree)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11 -mssse3)

set(LIBRARY_NAME ${PROJECT_NAME})
cs_add_library(${LIBRARY_NAME}
               src/helpers.cc
               src/vocabulary-tree-maker.cc)

catkin_add_gtest(test_vt_binary_descriptor test/test_binary-descriptor.cc
                 WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
target_link_libraries(test_vt_binary_descriptor
                      ${LIBRARY_NAME})

catkin_add_gtest(test_vt_binary_kmeans test/test_binary-kmeans.cc
                 WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
target_link_libraries(test_vt_binary_kmeans
                      ${LIBRARY_NAME})

catkin_add_gtest(test_vt_accelerated_quantization test/test_accelerated-quantization.cc
                 WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
target_link_libraries(test_vt_accelerated_quantization
                      ${LIBRARY_NAME})

catkin_add_gtest(test_vt_accelerated_kmeans test/test_accelerated-kmeans.cc
                 WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
target_link_libraries(test_vt_accelerated_kmeans
                      ${LIBRARY_NAME})

catkin_add_gtest(test_vt_binary_tree_builder test/test_binary-tree-builder.cc
                 WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
target_link_libraries(test_vt_binary_tree_builder
                      ${LIBRARY_NAME})

catkin_add_gtest(test_vt_binary_tree_serialization test/test_binary-tree-serialization.cc
                 WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
target_link_libraries(test_vt_binary_tree_serialization
                      ${LIBRARY_NAME})

catkin_add_gtest(test_vt_bucketized_tree test/test_bucketized-tree.cc
                 WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
target_link_libraries(test_vt_bucketized_tree
                      ${LIBRARY_NAME})


# CMake Indexing
FILE(GLOB_RECURSE LibFiles "include/*")
add_custom_target(headers SOURCES ${LibFiles})

cs_install()
cs_export(CFG_EXTRAS export_flags.cmake)
