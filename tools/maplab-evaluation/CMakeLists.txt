cmake_minimum_required (VERSION 2.8)
project(maplab_evaluation)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

catkin_python_setup()

# Tests.
catkin_add_nosetests(test/evaluation_test.py)

cs_install()
cs_export()
