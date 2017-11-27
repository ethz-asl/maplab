#include <random>

#include <gtest/gtest.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <sensors/relative-6dof-pose.h>

#include "sensors/gps-utm.h"
#include "sensors/gps-wgs.h"
#include "sensors/imu.h"
#include "sensors/measurements.pb.h"
#include "sensors/sensor-factory.h"
#include "sensors/sensor-system.h"

namespace vi_map {

TEST(SensorSystemTest, TestConstructor) {
  // Creating a sensor system with an invalid reference sensor id.
  SensorId reference_sensor_id;
  EXPECT_DEATH(SensorSystem sensor_system(reference_sensor_id), "");

  // Creating a sensor system with a valid reference sensor id.
  common::generateId(&reference_sensor_id);
  SensorSystem sensor_system(reference_sensor_id);
  const SensorSystemId sensor_system_id = sensor_system.getId();
  EXPECT_TRUE(sensor_system_id.isValid());

  // Testing that the sensor system contains exactly one id, namely the one of
  // the reference sensor.
  SensorIdSet sensor_ids;
  sensor_system.getAllSensorIds(&sensor_ids);
  ASSERT_EQ(sensor_ids.size(), 1u);
  EXPECT_EQ(*sensor_ids.begin(), reference_sensor_id);

  // Trying to create extriniscs with an invalid type.
  aslam::Transformation T_R_S;
  T_R_S.setRandom();
  ExtrinsicsType extrinsics_type = ExtrinsicsType::kInvalid;
  EXPECT_DEATH(Extrinsics(extrinsics_type, T_R_S), "");

  // Trying to create transformation extrinsics with an invalid sensor id.
  extrinsics_type = ExtrinsicsType::kTransformation;
  Extrinsics extrinsics(extrinsics_type, T_R_S);
  SensorId sensor_id;
  EXPECT_DEATH(sensor_system.setSensorExtrinsics(sensor_id, extrinsics), "");

  // Trying to create transformation extrinsics with a valid sensor id.
  common::generateId(&sensor_id);
  sensor_system.setSensorExtrinsics(sensor_id, extrinsics);

  // Testing that the sensor system now contains exactly 2 sensors, namely
  // the reference sensor, and the sensor with the transformation extrinsics.
  sensor_system.getAllSensorIds(&sensor_ids);
  EXPECT_EQ(sensor_ids.size(), 2u);
  EXPECT_GT(sensor_ids.count(sensor_id), 0u);
  EXPECT_GT(sensor_ids.count(reference_sensor_id), 0u);
  // Verifying the correctness of the extrinsics type and values.
  EXPECT_EQ(
      sensor_system.getSensorExtrinsicsType(sensor_id),
      ExtrinsicsType::kTransformation);
  EXPECT_EQ(sensor_system.getSensor_p_R_S(sensor_id), T_R_S.getPosition());
  EXPECT_EQ(sensor_system.getSensor_T_R_S(sensor_id), T_R_S);

  // Trying to add position-only extrinsics with a non-identity rotation.
  SensorId second_sensod_id;
  common::generateId(&second_sensod_id);
  aslam::Transformation T_R_S_second_sensor;
  T_R_S_second_sensor.setRandom();
  const ExtrinsicsType extrinsics_type_second_sensor =
      ExtrinsicsType::kPositionOnly;
  EXPECT_DEATH(
      Extrinsics(extrinsics_type_second_sensor, T_R_S_second_sensor), "");
  // Adding a position-only extrinsics with identity rotation.
  T_R_S_second_sensor.getRotation().setIdentity();
  sensor_system.setSensorExtrinsics(
      second_sensod_id,
      Extrinsics(extrinsics_type_second_sensor, T_R_S_second_sensor));

  // Verifying to the correctness of the position type and extrinsics values.
  EXPECT_EQ(
      sensor_system.getSensorExtrinsicsType(second_sensod_id),
      ExtrinsicsType::kPositionOnly);
  EXPECT_EQ(
      sensor_system.getSensor_p_R_S(second_sensod_id),
      T_R_S_second_sensor.getPosition());
  EXPECT_EQ(
      sensor_system.getSensor_T_R_S(second_sensod_id), T_R_S_second_sensor);

  // Trying to access inexistent sensor extrinsics.
  common::generateId(&sensor_id);
  EXPECT_DEATH(sensor_system.getSensorExtrinsicsType(sensor_id), "");
  EXPECT_DEATH(sensor_system.getSensor_T_R_S(sensor_id), "");
  EXPECT_DEATH(sensor_system.getSensor_p_R_S(sensor_id), "");
}
}  // namespace vi_map

MAPLAB_UNITTEST_ENTRYPOINT
