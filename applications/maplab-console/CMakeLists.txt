cmake_minimum_required (VERSION 2.8)
project(maplab_console)

find_package(catkin_simple REQUIRED)
catkin_simple()

cs_add_library(${PROJECT_NAME}_lib src/maplab-console.cc)
target_link_libraries(${PROJECT_NAME}_lib dl readline)

cs_add_executable(maplab_console src/maplab-console-app.cc)
target_link_libraries(maplab_console ${PROJECT_NAME}_lib)

cs_add_executable(batch_runner src/batch-runner.cc)
target_link_libraries(batch_runner ${PROJECT_NAME}_lib)

cs_install()
cs_export()
