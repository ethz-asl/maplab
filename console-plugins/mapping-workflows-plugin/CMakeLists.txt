cmake_minimum_required (VERSION 2.8)
project(mapping_workflows_plugin)

find_package(catkin_simple REQUIRED)
catkin_simple()

add_definitions(-fPIC -shared)

cs_add_library(${PROJECT_NAME}
  src/localization-map-creation.cc
  src/mapping-workflows-plugin.cc
)
create_console_plugin(${PROJECT_NAME})

cs_install()
cs_export()
