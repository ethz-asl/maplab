cmake_minimum_required(VERSION 2.8.3)
project(console_common)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############
find_package (Threads)
cs_add_library(${PROJECT_NAME} src/basic-console-plugin.cc
                               src/command-registerer.cc
                               src/console-plugin-base.cc
                               src/console.cc)
target_link_libraries(${PROJECT_NAME} readline)

############
## EXPORT ##
############
cs_install()
cs_export(CFG_EXTRAS create-console-plugin.cmake)

catkin_add_env_hooks(40.console-plugin-common
                     SHELLS sh
                     DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
