# This is demo on how to link libpointmatcher to an
# external project.

cmake_minimum_required (VERSION 2.8)
project(convert)

set (CMAKE_CXX_STANDARD 11)
add_compile_options(-std=c++11)

find_package(libpointmatcher REQUIRED)

include_directories(${CMAKE_CURRENT_BINARY_DIR} ${libpointmatcher_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} convert.cpp)

target_link_libraries(${PROJECT_NAME} ${libpointmatcher_LIBRARIES})
