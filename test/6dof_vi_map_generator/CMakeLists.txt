cmake_minimum_required(VERSION 2.8)
project(6dof_vi_map_generator)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}_lib src/6dof-pose-graph-gen.cc
                                   src/6dof-test-trajectory-gen.cc
                                   src/6dof-vi-map-gen.cc)

cs_install()
cs_export()
