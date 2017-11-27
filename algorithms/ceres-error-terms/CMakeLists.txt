cmake_minimum_required(VERSION 2.8.3)
project(ceres_error_terms)

link_directories(${CATKIN_DEVEL_PREFIX}/lib)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11 -Wno-enum-compare)

cs_add_library(${PROJECT_NAME}
  src/block-pose-prior-error-term.cc
  src/ceres-signal-handler.cc
  src/inertial-error-term.cc
  src/inertial-error-term-eigen.cc
  src/parameterization/quaternion-param-eigen.cc
  src/parameterization/quaternion-param-hamilton.cc
  src/parameterization/quaternion-param-jpl.cc
  src/parameterization/unit3-param.cc
  src/pose-prior-error-term-eigen.cc
  src/pose-prior-error-term.cc
  src/position-error-term.cc
  src/problem-information.cc)

target_link_libraries(${PROJECT_NAME} pthread)

catkin_add_gtest(test_quaternion_parameterization_test
  test/test_quaternion_parameterization_test.cc)
target_link_libraries(test_quaternion_parameterization_test ${PROJECT_NAME})

catkin_add_gtest(test_position_error_term
  test/test_position_error_term.cc)
target_link_libraries(test_position_error_term ${PROJECT_NAME})

catkin_add_gtest(test_pose_parameterization
  test/test_pose_parameterization_test.cc)
target_link_libraries(test_pose_parameterization ${PROJECT_NAME})

catkin_add_gtest(test_few_points_quaternion_test
  test/test_few_points_quaternion_test.cc)
target_link_libraries(test_few_points_quaternion_test ${PROJECT_NAME})

catkin_add_gtest(test_generic_prior_error_term
  test/test_generic_prior_error_term.cc)
target_link_libraries(test_generic_prior_error_term ${PROJECT_NAME})

catkin_add_gtest(test_visual_term_test test/test_visual_term_test.cc)
target_link_libraries(test_visual_term_test ${PROJECT_NAME})

catkin_add_gtest(test_switchable_constraints_block_pose_test
  test/test_switchable_constraints_block_pose_test.cc)
target_link_libraries(test_switchable_constraints_block_pose_test ${PROJECT_NAME})

catkin_add_gtest(test_switchable_constraints_block_pose_baseframe_test
  test/test_switchable_constraints_block_pose_baseframe_test.cc)
target_link_libraries(test_switchable_constraints_block_pose_baseframe_test ${PROJECT_NAME})

catkin_add_gtest(test_inertial_term_test
  test/test_inertial_term_test.cc)
target_link_libraries(test_inertial_term_test ${PROJECT_NAME})

catkin_add_gtest(test_inertial_term_test_eigen
  test/test_inertial_term_test_eigen.cc)
target_link_libraries(test_inertial_term_test_eigen ${PROJECT_NAME})

catkin_add_gtest(test_3keyframe_inertial_term_test
  test/test_3keyframe_inertial_term_test.cc)
target_link_libraries(test_3keyframe_inertial_term_test ${PROJECT_NAME})

catkin_add_gtest(test_mission_baseframe_visual_term_test
  test/test_mission_baseframe_visual_term_test.cc)
target_link_libraries(test_mission_baseframe_visual_term_test ${PROJECT_NAME})

catkin_add_gtest(test_block_pose_prior_error_term test/test_block_pose_prior_error_term.cc)
target_link_libraries(test_block_pose_prior_error_term ${PROJECT_NAME})

catkin_add_gtest(test_pose_prior_error_term test/test_pose_prior_error_term.cc)
target_link_libraries(test_pose_prior_error_term ${PROJECT_NAME})

catkin_add_gtest(test_six_dof_block_transformation_error_term
  test/test_six_dof_block_transformation_error_term.cc)
target_link_libraries(test_six_dof_block_transformation_error_term
  ${PROJECT_NAME})

catkin_add_gtest(test_six_dof_block_transformation_error_term_with_extrinsics
  test/test_six_dof_block_transformation_error_term_with_extrinsics.cc)
target_link_libraries(test_six_dof_block_transformation_error_term_with_extrinsics ${PROJECT_NAME})

catkin_add_gtest(test_unit3_parameterization
  test/test_unit3_parameterization.cc)
target_link_libraries(test_unit3_parameterization ${PROJECT_NAME})

catkin_add_gtest(test_quaternion_parameterization_eigen
  test/test_quaternion_eigen_parameterization.cc)
target_link_libraries(test_quaternion_parameterization_eigen ${PROJECT_NAME})

catkin_add_gtest(test_aid_error_term
  test/test_aid_error_term.cc)
target_link_libraries(test_aid_error_term ${PROJECT_NAME})

cs_install()
cs_export()
