cmake_minimum_required (VERSION 2.8)
project(vi_map_basic_plugin)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-fPIC -shared)

cs_add_library(${PROJECT_NAME} src/vi-map-basic-plugin.cc)
create_console_plugin(${PROJECT_NAME})

##########
# GTESTS #
##########
catkin_add_gtest(test_map_manager_vi_map test/map-manager-vi-map-test.cc)
target_link_libraries(test_map_manager_vi_map ${PROJECT_NAME})

catkin_add_gtest(test_map_manager_file_io test/map-manager-file-io-test.cc)
target_link_libraries(test_map_manager_file_io ${PROJECT_NAME})

cs_install()
cs_export()
