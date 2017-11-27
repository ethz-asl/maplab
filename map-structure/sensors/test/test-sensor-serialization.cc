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

namespace vi_map {

constexpr char kSensorFileName[] = "sensor.yaml";

template <class Sensor>
void testYamlSerializationDeserialization() {
  typename Sensor::UniquePtr sensor = createTestSensor<Sensor>();
  ASSERT_TRUE(static_cast<bool>(sensor));

  sensor->serializeToFile(static_cast<std::string>(kSensorFileName));

  typename Sensor::UniquePtr deserialized_sensor =
      createFromYaml<Sensor>(static_cast<std::string>(kSensorFileName));
  CHECK(deserialized_sensor);

  EXPECT_EQ(*sensor, *deserialized_sensor);
}

template <class DerivedSensor>
void testYamlSerializationFactoryDeserialization() {
  typename Sensor::UniquePtr sensor = createTestSensor<DerivedSensor>();
  ASSERT_TRUE(static_cast<bool>(sensor));

  sensor->serializeToFile(static_cast<std::string>(kSensorFileName));
  Sensor::UniquePtr deserialized_sensor =
      createSensorFromYaml(static_cast<std::string>(kSensorFileName));
  CHECK(deserialized_sensor);
  typename DerivedSensor::UniquePtr deserialized_derived_sensor(
      static_cast<DerivedSensor*>(deserialized_sensor.release()));
  CHECK(deserialized_derived_sensor);
  EXPECT_EQ(*sensor, *deserialized_derived_sensor);
}

TEST(SensorsTest, YamlSeriazliation) {
  testYamlSerializationDeserialization<Imu>();
  testYamlSerializationFactoryDeserialization<Imu>();

  testYamlSerializationDeserialization<Relative6DoFPose>();
  testYamlSerializationFactoryDeserialization<Relative6DoFPose>();

  testYamlSerializationDeserialization<GpsUtm>();
  testYamlSerializationFactoryDeserialization<GpsUtm>();

  testYamlSerializationDeserialization<GpsWgs>();
  testYamlSerializationFactoryDeserialization<GpsWgs>();
}

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

}  // namespace vi_map

MAPLAB_UNITTEST_ENTRYPOINT
