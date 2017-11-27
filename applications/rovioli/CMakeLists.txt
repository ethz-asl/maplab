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
  src/feature-tracking.cc
  src/imu-camera-synchronizer.cc
  src/localizer.cc
  src/map-builder-flow.cc
  src/rovio-factory.cc
  src/rovio-flow.cc
  src/rovioli-node.cc
  src/synced-nframe-throttler.cc
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
catkin_add_gtest(test_localizer test/test-localizer.cc)
target_link_libraries(test_localizer ${PROJECT_NAME}_lib)
maplab_import_test_maps(test_localizer)

catkin_add_gtest(test_feature_tracking test/test-feature-tracking.cc)
target_link_libraries(test_feature_tracking ${PROJECT_NAME}_lib)
maplab_import_test_maps(test_feature_tracking)

catkin_add_gtest(test_vio_update_builder test/test-vio-update-builder.cc)
target_link_libraries(test_vio_update_builder ${PROJECT_NAME}_lib)

execute_process(COMMAND tar -xzf ${MAPLAB_TEST_DATA_DIR}/end_to_end_test/end_to_end_test.tar.gz)
catkin_add_nosetests(test/end-to-end-test.py)

############
## EXPORT ##
############
cs_install()
cs_export()
