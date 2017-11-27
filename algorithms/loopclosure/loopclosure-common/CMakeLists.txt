cmake_minimum_required(VERSION 2.8.3)
project(loopclosure_common)

find_package(catkin_simple REQUIRED)
catkin_simple()

set(LIBRARY_NAME ${PROJECT_NAME})
cs_add_library(${LIBRARY_NAME} src/flags.cc)

# CMake Indexing
FILE(GLOB_RECURSE LibFiles "include/*")
add_custom_target(headers SOURCES ${LibFiles})

cs_install()
cs_export()
