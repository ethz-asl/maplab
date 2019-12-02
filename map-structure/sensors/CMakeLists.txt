cmake_minimum_required(VERSION 2.8.3)
project(sensors)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME} ${PROTO_SRCS} ${PROTO_HDRS}
  src/absolute-6dof-pose.cc
  src/gps-utm.cc
  src/gps-wgs.cc
  src/imu.cc
  src/lidar.cc
  src/odometry-6dof-pose.cc
  src/pointcloud-map-sensor.cc
  src/wheel-odometry-sensor.cc
  src/loop-closure-sensor.cc
)

##########
# GTESTS #
##########
catkin_add_gtest(test_sensor_serialization test/test-sensor-serialization.cc)
target_link_libraries(test_sensor_serialization ${PROJECT_NAME})

catkin_add_gtest(test_sensors test/test-sensors.cc)
target_link_libraries(test_sensors ${PROJECT_NAME})

catkin_add_gtest(test_measurements test/test-measurements.cc)
target_link_libraries(test_measurements ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
