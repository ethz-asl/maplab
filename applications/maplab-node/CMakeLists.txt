cmake_minimum_required(VERSION 2.8.3)
project(maplab_node)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(VOXGRAPH_ENABLED FALSE)
if(VOXGRAPH_ENABLED)
  find_package(voxgraph_msgs)
  if (voxgraph_msgs_FOUND)
    message("Compiling maplab_node WITH voxgraph_msgs support!")
    add_definitions(-DVOXGRAPH)
  else()
  	message(ERROR "Could not find voxgraph_msgs, compiling maplab_node WITHOUT voxgraph_msgs support!")
  endif()
else()
  message("Compiling maplab_node WITHOUT voxgraph_msgs support!")
endif()

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
  src/synchronizer.cc
	src/localization-handler-flow.cc
	src/localization-handler.cc
	src/map-builder-flow.cc
	src/maplab-node.cc
	src/maplab-ros-node.cc
	src/ros-helpers.cc
  src/map-update-builder.cc
	src/visual-localizer-flow.cc
	src/visual-localizer-helpers.cc
	src/visual-localizer.cc)


#######
# APP #
#######
cs_add_executable(${PROJECT_NAME}
  app/maplab-ros-node-app.cc
)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)

##########
# GTESTS #
##########
catkin_add_gtest(test_feature_tracking test/test-feature-tracking.cc)
target_link_libraries(test_feature_tracking ${PROJECT_NAME}_lib)
maplab_import_test_maps(test_feature_tracking)

catkin_add_gtest(test_map_update_builder test/test-map-update-builder.cc)
target_link_libraries(test_map_update_builder ${PROJECT_NAME}_lib)

execute_process(COMMAND tar -xzf ${MAPLAB_TEST_DATA_DIR}/end_to_end_test/end_to_end_test.tar.gz)
catkin_add_nosetests(test/end_to_end_test.py)

############
## EXPORT ##
############
cs_install()
cs_export()
