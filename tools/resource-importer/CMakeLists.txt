cmake_minimum_required(VERSION 2.8)
project(resource_importer)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}_lib src/message-conversion.cc
                                   src/simple-rosbag-reader.cc)

cs_add_executable(${PROJECT_NAME} role/resource-importer.cc)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)

cs_install_scripts(scripts/import_resources_w_camera_info.sh
                   scripts/import_resources_w_ncamera_yaml.sh)

cs_install()
cs_export()
