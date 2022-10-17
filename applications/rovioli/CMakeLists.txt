cmake_minimum_required(VERSION 2.8.3)
project(rovioli)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}_lib
  src/data-publisher-flow.cc
  src/datasource-factory.cc
  src/datasource-rosbag.cc
  src/datasource-rostopic.cc
  src/datasource.cc
  src/feature-tracking.cc
  src/imu-camera-synchronizer.cc
  src/localizer-flow.cc
  src/localizer-helpers.cc
  src/localizer.cc
  src/map-builder-flow.cc
  src/ros-helpers.cc
  src/rovio-factory.cc
  src/rovio-flow.cc
  src/rovio-localization-handler.cc
  src/rovioli-node.cc
  src/vio-update-builder.cc
)

#######
# APP #
#######
cs_add_executable(rovioli
  app/rovioli-app.cc
)
target_link_libraries(rovioli ${PROJECT_NAME}_lib)

#########
# SHARE #
#########
catkin_add_env_hooks(10.rovio-config-template
  SHELLS sh
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks
)

##########
# GTESTS #
##########
catkin_add_gtest(test_feature_tracking test/test-feature-tracking.cc)
target_link_libraries(test_feature_tracking ${PROJECT_NAME}_lib)
maplab_import_test_maps(test_feature_tracking)

catkin_add_gtest(test_vio_update_builder test/test-vio-update-builder.cc)
target_link_libraries(test_vio_update_builder ${PROJECT_NAME}_lib)

catkin_add_gtest(test_ros_helpers test/test-ros-helpers.cc)
target_link_libraries(test_ros_helpers ${PROJECT_NAME}_lib)

############
## EXPORT ##
############
cs_install()
cs_export()
