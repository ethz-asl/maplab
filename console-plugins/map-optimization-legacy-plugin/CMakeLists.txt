cmake_minimum_required (VERSION 2.8)
project(map_optimization_legacy_plugin)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-fPIC -shared)

cs_add_library(${PROJECT_NAME} src/legacy-optimizer-plugin.cc
                               src/vi-map-optimizer-legacy.cc)
create_console_plugin(${PROJECT_NAME})

#############
## TESTING ##
#############
catkin_add_gtest(test_optimization test/test-optimization.cc)
target_link_libraries(test_optimization ${PROJECT_NAME})
maplab_import_test_maps(test_optimization)

cs_install()
cs_export()
