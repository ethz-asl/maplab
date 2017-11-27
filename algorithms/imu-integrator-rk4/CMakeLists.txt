cmake_minimum_required(VERSION 2.8.3)
project(imu_integrator_rk4)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}
  src/imu-integrator.cc
  src/imu-integrator-eigen.cc
)
target_link_libraries(${PROJECT_NAME} pthread)

catkin_add_gtest(test_imu_integrator_basic_test
  test/test_imu_integrator_basic_test.cc)
target_link_libraries(test_imu_integrator_basic_test ${PROJECT_NAME})

catkin_add_gtest(test_imu_integrator_trajectory_test
  test/test_imu_integrator_trajectory_test.cc)
target_link_libraries(test_imu_integrator_trajectory_test ${PROJECT_NAME})

catkin_add_gtest(test_imu_integrator_cov_phi_test
  test/test_imu_integrator_cov_phi_test.cc)
target_link_libraries(test_imu_integrator_cov_phi_test ${PROJECT_NAME})

catkin_add_gtest(test_imu_integrator_eigen_basic_test
  test/test_imu_integrator_eigen_basic_test.cc)
target_link_libraries(test_imu_integrator_eigen_basic_test ${PROJECT_NAME})

cs_install()
cs_export()
