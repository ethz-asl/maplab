cmake_minimum_required(VERSION 2.8.3)
project(feature_tracking_pipelines_driver)
add_compile_options(-w)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_executable(feature_tracking_app app/feature-tracking-app.cc)

##########
# GTESTS #
##########
#catkin_add_gtest(test_feature_extractor
#  test/test-feature-track-extractor.cc
#)
#target_link_libraries(test_feature_extractor ${PROJECT_NAME})

cs_install()
cs_export()
