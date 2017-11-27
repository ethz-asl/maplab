cmake_minimum_required (VERSION 2.8)
project(vi_mapping_test_app)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(--std=c++11 -Wno-unknown-pragmas)

cs_add_library(${PROJECT_NAME} src/vi-mapping-test-app.cc)
if(APPLE)
  target_link_libraries(${PROJECT_NAME} -lgtest)
endif()

cs_install()
cs_export()
