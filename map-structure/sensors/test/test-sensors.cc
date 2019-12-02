#include <random>

#include <gtest/gtest.h>
#include <maplab-common/string-tools.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

#include <sensors/absolute-6dof-pose.h>
#include <sensors/gps-utm.h>
#include <sensors/gps-wgs.h>
#include <sensors/imu.h>
#include <sensors/lidar.h>
#include <sensors/loop-closure-sensor.h>
#include <sensors/odometry-6dof-pose.h>
#include <sensors/pointcloud-map-sensor.h>
#include <sensors/wheel-odometry-sensor.h>

namespace vi_map {

TEST(SensorsTest, TestSetters) {
  Imu imu;
  imu.setRandom();

  // Testing sensor id getter and setter.
  aslam::SensorId sensor_id;
  EXPECT_DEATH(imu.setId(sensor_id), "");
  aslam::generateId(&sensor_id);
  EXPECT_NE(imu.getId(), sensor_id);
  imu.setId(sensor_id);
  EXPECT_EQ(imu.getId(), sensor_id);

  // Testing IMU sigmas getter and setter.
  ImuSigmas imu_sigmas;
  imu_sigmas.setRandom();
  EXPECT_FALSE(imu.getImuSigmas() == imu_sigmas);
  imu.setImuSigmas(imu_sigmas);
  EXPECT_EQ(imu.getImuSigmas(), imu_sigmas);

  // Testing gravity magnitude getter and setter.
  std::random_device random_device;
  std::default_random_engine random_engine(random_device());
  std::uniform_real_distribution<double> uniform_gravity(
      Imu::kMinGravity, Imu::kMaxGravity);
  const double gravity_magnitude = uniform_gravity(random_engine);
  EXPECT_NE(imu.getGravityMagnitudeMps2(), gravity_magnitude);
  imu.setGravityMagnitude(gravity_magnitude);
  EXPECT_EQ(imu.getGravityMagnitudeMps2(), gravity_magnitude);
}

TEST(SensorsTest, TestSensorConstructor) {
  aslam::SensorId sensor_id;

  // Testing constructor with invalid sensor id.
  const std::string kEmptyTopic = "";
  EXPECT_DEATH(aligned_unique<Imu>(sensor_id, kEmptyTopic), "id.isValid()");

  aslam::generateId(&sensor_id);

  constexpr size_t kTopicLength = 20u;
  const std::string topic = common::createRandomString(kTopicLength);

  // Testing constructor with invalid sensor id.
  sensor_id.setInvalid();
  EXPECT_DEATH(aligned_unique<Imu>(sensor_id, topic), "");

  // Testing constructor with valid sensor and hardware id.
  aslam::generateId(&sensor_id);
  Imu::UniquePtr sensor = aligned_unique<Imu>(sensor_id, topic);
  EXPECT_EQ(sensor->getId(), sensor_id);
  EXPECT_EQ(sensor->getTopic(), topic);
}

TEST(SensorsTest, TestRel6DoFConstructor) {
  aslam::SensorId sensor_id;
  aslam::generateId(&sensor_id);
  constexpr size_t kTopicLength = 20u;
  const std::string topic = common::createRandomString(kTopicLength);

  // Testing sensor and hardware id, and pose measurement covariance getters.
  LoopClosureSensor::UniquePtr sensor =
      aligned_unique<LoopClosureSensor>(sensor_id, topic);
  EXPECT_EQ(sensor->getId(), sensor_id);
  EXPECT_EQ(sensor->getTopic(), topic);
}

TEST(SensorsTest, TestGpsUtm) {
  aslam::SensorId sensor_id;
  aslam::generateId(&sensor_id);
  constexpr size_t kTopicLength = 20u;
  const std::string topic = common::createRandomString(kTopicLength);

  // Testing sensor and hardware id getters.
  GpsUtm::UniquePtr sensor = aligned_unique<GpsUtm>(sensor_id, topic);
  EXPECT_EQ(sensor->getId(), sensor_id);
  EXPECT_EQ(sensor->getTopic(), topic);
}

TEST(SensorsTest, TestGpsWgs) {
  aslam::SensorId sensor_id;
  aslam::generateId(&sensor_id);
  constexpr size_t kTopicLength = 20u;
  const std::string topic = common::createRandomString(kTopicLength);

  // Testing sensor and hardware id getters.
  GpsWgs::UniquePtr sensor = aligned_unique<GpsWgs>(sensor_id, topic);
  EXPECT_EQ(sensor->getId(), sensor_id);
  EXPECT_EQ(sensor->getTopic(), topic);
}

TEST(SensorsTest, TestLidar) {
  aslam::SensorId sensor_id;
  aslam::generateId(&sensor_id);
  constexpr size_t kTopicLength = 20u;
  const std::string topic = common::createRandomString(kTopicLength);

  // Testing sensor and hardware id getters.
  Lidar::UniquePtr sensor = aligned_unique<Lidar>(sensor_id, topic);
  EXPECT_EQ(sensor->getId(), sensor_id);
  EXPECT_EQ(sensor->getTopic(), topic);
}

TEST(SensorsTest, TestOdometry6DoF) {
  aslam::SensorId sensor_id;
  aslam::generateId(&sensor_id);
  constexpr size_t kTopicLength = 20u;
  const std::string topic = common::createRandomString(kTopicLength);

  // Testing sensor and hardware id getters.
  Odometry6DoF::UniquePtr sensor =
      aligned_unique<Odometry6DoF>(sensor_id, topic);
  EXPECT_EQ(sensor->getId(), sensor_id);
  EXPECT_EQ(sensor->getTopic(), topic);
}

TEST(SensorsTest, TestAbsolute6DoF) {
  aslam::SensorId sensor_id;
  aslam::generateId(&sensor_id);
  constexpr size_t kTopicLength = 20u;
  const std::string topic = common::createRandomString(kTopicLength);

  // Testing sensor and hardware id getters.
  Absolute6DoF::UniquePtr sensor =
      aligned_unique<Absolute6DoF>(sensor_id, topic);
  EXPECT_EQ(sensor->getId(), sensor_id);
  EXPECT_EQ(sensor->getTopic(), topic);
}

TEST(SensorsTest, TestPointCloudMapSensor) {
  aslam::SensorId sensor_id;
  aslam::generateId(&sensor_id);
  constexpr size_t kTopicLength = 20u;
  const std::string topic = common::createRandomString(kTopicLength);

  // Testing sensor and hardware id getters.
  PointCloudMapSensor::UniquePtr sensor =
      aligned_unique<PointCloudMapSensor>(sensor_id, topic);
  EXPECT_EQ(sensor->getId(), sensor_id);
  EXPECT_EQ(sensor->getTopic(), topic);
}

}  // namespace vi_map

MAPLAB_UNITTEST_ENTRYPOINT
