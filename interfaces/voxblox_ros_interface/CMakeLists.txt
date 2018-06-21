cmake_minimum_required (VERSION 2.8)
project(voxblox_ros_interface)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(--std=c++11)

cs_add_library(${PROJECT_NAME} src/voxblox_bag_importer.cc)

cs_add_executable(voxblox_bag_importer src/voxblox_bag_importer_node.cc)
target_link_libraries(voxblox_bag_importer ${PROJECT_NAME})

cs_install()
cs_export()
