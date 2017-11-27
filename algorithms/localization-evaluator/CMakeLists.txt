cmake_minimum_required(VERSION 2.8.3)
project(localization_evaluator)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME} 
  src/localization-evaluator.cc
  src/mission-aligner.cc)
  
catkin_add_gtest(test_localization_evaluator
  test/test_localization_evaluator.cc)
target_link_libraries(test_localization_evaluator ${PROJECT_NAME})
maplab_import_test_maps(test_localization_evaluator)

cs_install()
cs_export()
