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

TEST(SensorsTest, TestSetters) {
  Imu::UniquePtr imu = createTestSensor<Imu>();
  ASSERT_TRUE(static_cast<bool>(imu));

  // Testing sensor id getter and setter.
  SensorId sensor_id;
  EXPECT_DEATH(imu->setId(sensor_id), "");
  common::generateId(&sensor_id);
  EXPECT_NE(imu->getId(), sensor_id);
  imu->setId(sensor_id);
  EXPECT_EQ(imu->getId(), sensor_id);

  // Testing IMU sigmas getter and setter.
  ImuSigmas imu_sigmas;
  imu_sigmas.setRandom();
  EXPECT_FALSE(imu->getImuSigmas() == imu_sigmas);
  imu->setImuSigmas(imu_sigmas);
  EXPECT_EQ(imu->getImuSigmas(), imu_sigmas);

  // Testing gravity magnitude getter and setter.
  std::random_device random_device;
  std::default_random_engine random_engine(random_device());
  std::uniform_real_distribution<double> uniform_gravity(
      Imu::kMinGravity, Imu::kMaxGravity);
  const double gravity_magnitude = uniform_gravity(random_engine);
  EXPECT_NE(imu->getGravityMagnitudeMps2(), gravity_magnitude);
  imu->setGravityMagnitude(gravity_magnitude);
  EXPECT_EQ(imu->getGravityMagnitudeMps2(), gravity_magnitude);
}

TEST(SensorsTest, TestSensorConstructor) {
  SensorId sensor_id;
  common::generateId(&sensor_id);

  // Testing constructor with empty hardware id.
  const std::string kEmptyHardwareId = "";
  EXPECT_DEATH(
      aligned_unique<Imu>(sensor_id, kEmptyHardwareId),
      "A sensor needs a non-empty hardware identification label.");

  constexpr size_t kHardwareIdLength = 20u;
  const std::string hardware_id = common::createRandomString(kHardwareIdLength);

  // Testing constructor with invalid sensor id.
  sensor_id.setInvalid();
  EXPECT_DEATH(aligned_unique<Imu>(sensor_id, hardware_id), "");

  // Testing constructor with valid sensor and hardware id.
  common::generateId(&sensor_id);
  Imu::UniquePtr sensor = aligned_unique<Imu>(sensor_id, hardware_id);
  EXPECT_EQ(sensor->getId(), sensor_id);
  EXPECT_EQ(sensor->getHardwareId(), hardware_id);
}

TEST(SensorsTest, TestRel6DoFConstructor) {
  SensorId sensor_id;
  common::generateId(&sensor_id);
  constexpr size_t kHardwareIdLength = 20u;
  const std::string hardware_id = common::createRandomString(kHardwareIdLength);
  aslam::TransformationCovariance pose_measurement_covariance;
  pose_measurement_covariance.setRandom();

  // Testing sensor and hardware id, and pose measurement covariance getters.
  Relative6DoFPose::UniquePtr sensor = aligned_unique<Relative6DoFPose>(
      sensor_id, hardware_id, pose_measurement_covariance);
  EXPECT_EQ(sensor->getId(), sensor_id);
  EXPECT_EQ(sensor->getHardwareId(), hardware_id);
  EXPECT_EQ(
      sensor->getPoseMeasurementCovariance(), pose_measurement_covariance);
}

TEST(SensorsTest, TestGpsUtm) {
  SensorId sensor_id;
  common::generateId(&sensor_id);
  constexpr size_t kHardwareIdLength = 20u;
  const std::string hardware_id = common::createRandomString(kHardwareIdLength);

  // Testing sensor and hardware id getters.
  GpsUtm::UniquePtr sensor = aligned_unique<GpsUtm>(sensor_id, hardware_id);
  EXPECT_EQ(sensor->getId(), sensor_id);
  EXPECT_EQ(sensor->getHardwareId(), hardware_id);
}

TEST(SensorsTest, TestGpsWgs) {
  SensorId sensor_id;
  common::generateId(&sensor_id);
  constexpr size_t kHardwareIdLength = 20u;
  const std::string hardware_id = common::createRandomString(kHardwareIdLength);

  // Testing sensor and hardware id getters.
  GpsWgs::UniquePtr sensor = aligned_unique<GpsWgs>(sensor_id, hardware_id);
  EXPECT_EQ(sensor->getId(), sensor_id);
  EXPECT_EQ(sensor->getHardwareId(), hardware_id);
}
}  // namespace vi_map

MAPLAB_UNITTEST_ENTRYPOINT
