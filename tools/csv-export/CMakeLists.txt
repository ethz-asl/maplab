cmake_minimum_required(VERSION 2.8.3)
project(csv_export)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(--std=c++11)

SET(SRCS src/csv-export.cc)
cs_add_library(${PROJECT_NAME} ${SRCS})

##########
# EXPORT #
##########
cs_install()
cs_export()
