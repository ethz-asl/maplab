cmake_minimum_required(VERSION 2.8.3)
project(landmark_triangulation)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME} 
  src/landmark-triangulation.cc
  src/pose-interpolator.cc
)

catkin_add_gtest(test_pose_interpolator_test
  test/test_pose_interpolator_test.cc)
target_link_libraries(test_pose_interpolator_test ${PROJECT_NAME})

catkin_add_gtest(test_landmark_triangulation test/test_landmark_triangulation.cc)
target_link_libraries(test_landmark_triangulation ${PROJECT_NAME})
maplab_import_test_maps(test_landmark_triangulation)

cs_install()
cs_export()
