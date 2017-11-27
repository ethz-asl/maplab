cmake_minimum_required(VERSION 2.8.3)
project(geometric_vision_algorithms)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(--std=c++11)

cs_add_library(${PROJECT_NAME}
  src/five-point-pose-estimator.cc
  src/linear-triangulation.cc
  src/relative-non-central-pnp.cc
  src/rotation-only-detector.cc)

catkin_add_gtest(test_five_point_pose_estimator_test
  test/test_five_point_pose_estimator_test.cc)
target_link_libraries(test_five_point_pose_estimator_test ${PROJECT_NAME})

catkin_add_gtest(test_n_view_triangulation
	test/test_n_view_triangulation.cc)
target_link_libraries(test_n_view_triangulation ${PROJECT_NAME})

catkin_add_gtest(test_pnp_pose_test
	test/test_pnp_pose_test.cc)
target_link_libraries(test_pnp_pose_test ${PROJECT_NAME})

catkin_add_gtest(test_relative_non_central_pnp
	test/test_relative_non_central_pnp.cc)
target_link_libraries(test_relative_non_central_pnp ${PROJECT_NAME} )

catkin_add_gtest(test_rotation_only_detector
  test/test_rotation_only_detector.cc)
target_link_libraries(test_rotation_only_detector ${PROJECT_NAME})

catkin_add_gtest(test_two_view_triangulation
	test/test_two_view_triangulation.cc)
target_link_libraries(test_two_view_triangulation ${PROJECT_NAME})

cs_install()
cs_export()
