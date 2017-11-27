cmake_minimum_required(VERSION 2.8.3)
project(map_manager)

find_package(catkin_simple REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(--std=c++11)

#############
# LIBRARIES #
#############
# Core Library available to all applications
SET(CORE_SOURCE src/dummy-src.cc)

cs_add_library(${PROJECT_NAME} ${CORE_SOURCE})

##########
# GTESTS #
##########
catkin_add_gtest(test_map_manager_basic test/map-manager-basic-test.cc)
target_link_libraries(test_map_manager_basic ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
