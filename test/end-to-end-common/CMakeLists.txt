cmake_minimum_required (VERSION 2.8)
project(end_to_end_common)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

catkin_python_setup()

cs_install()
cs_export()
