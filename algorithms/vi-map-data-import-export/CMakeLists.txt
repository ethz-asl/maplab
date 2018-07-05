cmake_minimum_required (VERSION 2.8)
project(vi_map_data_import_export)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-fPIC -shared)

cs_add_library(${PROJECT_NAME} src/import-export-gps-data.cc 
                               src/export-ncamera-calibration.cc
                               src/export-vertex-data.cc)

cs_install()
cs_export()
