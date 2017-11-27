cmake_minimum_required (VERSION 2.8)
project(pmvs_interface)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(--std=c++11 -Wno-unknown-pragmas)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

cs_add_library(${PROJECT_NAME}  src/pmvs-common.cc
                                src/pmvs-config.cc
                                src/pmvs-file-utils.cc
                                src/pmvs-interface.cc)

catkin_add_gtest(test_pmvs_interface test/test_pmvs_interface.cc)
target_link_libraries(test_pmvs_interface ${PROJECT_NAME})

cs_install()
cs_export()
