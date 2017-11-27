cmake_minimum_required (VERSION 2.8)
project(visualization)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(--std=c++11 -Wno-unknown-pragmas)

##########################
# EXECUTABLE AND LIBRARY #
##########################
cs_add_library(${PROJECT_NAME}
  src/color-palette.cc
  src/common-rviz-visualization.cc
  src/constant-velocity-smoother.cc
  src/debug-visualizer.cc
  src/feature-matches-visualization.cc
  src/patch-based-visualization.cc
  src/rviz-visualization-sink.cc
  src/sequential-plotter.cc
  src/sliding-window-plotter.cc
  src/vertex-landmark-visibility-plotter.cc
  src/viwls-graph-plotter.cc
  src/viz-data-collector.cc
  src/visualizer.cc
)

##########
# GTESTS #
##########
catkin_add_gtest(test_viz_channels test/test-viz-channels.cc)
target_link_libraries(test_viz_channels ${PROJECT_NAME})

catkin_add_gtest(test_viz_data_collector test/test-viz-data-collector.cc)
target_link_libraries(test_viz_data_collector ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
