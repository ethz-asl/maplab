#include <random>

#include <gtest/gtest.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <sensors/relative-6dof-pose.h>

#include "sensors/gps-utm.h"
#include "sensors/gps-wgs.h"
#include "sensors/imu.h"
#include "sensors/lidar.h"
#include "sensors/measurements.pb.h"

namespace vi_map {

TEST(MeasurementsTest, GpsUtmProtoSerialization) {
  GpsUtmMeasurement gps_utm_measurement(common::createRandomId<SensorId>());
  gps_utm_measurement.setRandom();

  measurements::proto::GpsUtmMeasurement proto_gps_utm_measurement;
  gps_utm_measurement.serialize(&proto_gps_utm_measurement);

  GpsUtmMeasurement deserialized_gps_utm_measurement;
  deserialized_gps_utm_measurement.deserialize(proto_gps_utm_measurement);

  EXPECT_EQ(gps_utm_measurement, deserialized_gps_utm_measurement);
}

TEST(MeasurementsTest, GpsWgsProtoSerialization) {
  GpsWgsMeasurement gps_wgs_measurement(common::createRandomId<SensorId>());
  gps_wgs_measurement.setRandom();

  measurements::proto::GpsWgsMeasurement proto_gps_wgs_measurement;
  gps_wgs_measurement.serialize(&proto_gps_wgs_measurement);

  GpsWgsMeasurement deserialized_gps_wgs_measurement;
  deserialized_gps_wgs_measurement.deserialize(proto_gps_wgs_measurement);

  EXPECT_EQ(gps_wgs_measurement, deserialized_gps_wgs_measurement);
}

TEST(MeasurementsTest, ImuMeasurementProtoSerialization) {
  ImuMeasurement imu_measurement(common::createRandomId<SensorId>());
  imu_measurement.setRandom();

  measurements::proto::ImuMeasurement proto_imu_measurement;
  imu_measurement.serialize(&proto_imu_measurement);

  ImuMeasurement deserialized_imu_measurement;
  deserialized_imu_measurement.deserialize(proto_imu_measurement);

  EXPECT_EQ(imu_measurement, deserialized_imu_measurement);
}

}  // namespace vi_map

MAPLAB_UNITTEST_ENTRYPOINT
