cmake_minimum_required(VERSION 2.8.3)
project(dense_reconstruction_plugin)

find_package(catkin_simple REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-fPIC -shared)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME} src/dense-reconstruction-plugin.cc)
create_console_plugin(${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
