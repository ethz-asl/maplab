cmake_minimum_required(VERSION 2.8.3)
project(sensors)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

############
# PROTOBUF #
############
set(PROTO_DEFNS proto/sensors/measurements.proto)
set(BASE_PATH "proto")
PROTOBUF_CATKIN_GENERATE_CPP2(${BASE_PATH} PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS}) 

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME} ${PROTO_SRCS} ${PROTO_HDRS}
  src/gps-utm.cc
  src/gps-wgs.cc
  src/imu.cc
  src/measurement.cc
  src/relative-6dof-pose.cc
  src/sensor.cc
  src/sensor-extrinsics.cc
  src/sensor-factory.cc
  src/sensor-system.cc
  ${PROTO_SRCS}
  ${PROTO_HDRS}
)

##########
# GTESTS #
##########
catkin_add_gtest(test_serialization test/test-sensor-serialization.cc)
target_link_libraries(test_serialization ${PROJECT_NAME})

catkin_add_gtest(test_sensors test/test-sensors.cc)
target_link_libraries(test_sensors ${PROJECT_NAME})

catkin_add_gtest(test_sensor_system test/test-sensor-system.cc)
target_link_libraries(test_sensor_system ${PROJECT_NAME})

catkin_add_gtest(test_measurements test/test-measurements.cc)
target_link_libraries(test_measurements ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
