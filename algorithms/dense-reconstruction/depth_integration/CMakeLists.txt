cmake_minimum_required (VERSION 2.8)
project(depth_integration)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}
        src/depth-integration.cc
        src/depth-map-integration.cc)

SET(PROJECT_TEST_DATA "map_resources_test_data")
add_custom_target(${PROJECT_TEST_DATA})
add_custom_command(TARGET ${PROJECT_TEST_DATA}
COMMAND rm -rf "./${PROJECT_TEST_DATA}/*" && tar -xvzf ${MAPLAB_TEST_DATA_DIR}/${PROJECT_TEST_DATA}/${PROJECT_TEST_DATA}.tar.gz)

catkin_add_gtest(test_voxblox_depth_integration test/test-voxblox-depth-integration.cc)
target_link_libraries(test_voxblox_depth_integration ${PROJECT_NAME})
maplab_import_test_maps(test_voxblox_depth_integration)
add_dependencies(test_voxblox_depth_integration ${PROJECT_TEST_DATA})

cs_install()
cs_export()
