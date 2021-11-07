cmake_minimum_required (VERSION 2.8)
project(semantify_plugin)
set(CMAKE_CXX_STANDARD 11)

find_package(catkin_simple REQUIRED mask_rcnn_ros netvlad_tf cv_bridge deep_sort_ros)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}
    src/semantify-plugin.cc
    src/semantify-common.cc
    src/k-medoids-clustering-manager.cc
    src/hungarian-algorithm.cc
    src/hungarian-algorithm-manager.cc)
# target_link_libraries(${PROJECT_NAME})
create_console_plugin(${PROJECT_NAME})

##########
# GTESTS #
##########
catkin_add_gtest(test_map_semantic_landmark_triangulation_test
 test/test_map_semantic_landmark_triangulation_test.cc)
target_link_libraries(test_map_semantic_landmark_triangulation_test ${PROJECT_NAME})

##################
# CMake Indexing #
##################
FILE(GLOB_RECURSE LibFiles "include/*")
add_custom_target(headers SOURCES ${LibFiles})

cs_install()
cs_export()
