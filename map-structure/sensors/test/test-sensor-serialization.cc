#include <random>

#include <gtest/gtest.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <sensors/odometry-6dof-pose.h>

#include "sensors/gps-utm.h"
#include "sensors/gps-wgs.h"
#include "sensors/imu.h"
#include "sensors/lidar.h"
#include "sensors/loop-closure-sensor.h"
#include "sensors/wheel-odometry-sensor.h"

namespace vi_map {

constexpr char kSensorFileName[] = "sensor.yaml";

template <class DerivedSensor>
void testYamlSerializationDeserialization() {
  typename aslam::Sensor::UniquePtr sensor = aligned_unique<DerivedSensor>();
  sensor->setRandom();
  ASSERT_TRUE(static_cast<bool>(sensor));

  sensor->serializeToFile(static_cast<std::string>(kSensorFileName));

  typename aslam::Sensor::UniquePtr deserialized_sensor =
      aligned_unique<DerivedSensor>();
  deserialized_sensor->deserializeFromFile(kSensorFileName);

  EXPECT_TRUE(sensor->isEqual(*deserialized_sensor, true /*verbose*/));
}

TEST(SensorsTest, YamlSeriazliation) {
  testYamlSerializationDeserialization<Imu>();
  testYamlSerializationDeserialization<LoopClosureSensor>();
  testYamlSerializationDeserialization<GpsUtm>();
  testYamlSerializationDeserialization<GpsWgs>();
  testYamlSerializationDeserialization<Lidar>();
  testYamlSerializationDeserialization<Odometry6DoF>();
  testYamlSerializationDeserialization<WheelOdometry>();
}

}  // namespace vi_map

MAPLAB_UNITTEST_ENTRYPOINT
