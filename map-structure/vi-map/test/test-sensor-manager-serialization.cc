#include <string>

#include <gtest/gtest.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

#include "vi-map/sensor-manager.h"
#include "vi-map/vi-map-serialization.h"

namespace vi_map {

class SensorManagerTest : public ::testing::Test {
 protected:
  virtual void SetUp() {}

  aslam::SensorId addImu() {
    Imu::UniquePtr imu(new Imu());
    CHECK(imu);
    imu->setRandom();
    CHECK(imu->isValid());
    const aslam::SensorId& base_id = imu->getId();
    aslam::Transformation T_B_S;
    T_B_S.setIdentity();
    sensor_manager_.addSensor<Imu>(std::move(imu), base_id, T_B_S);
    return base_id;
  }

  aslam::SensorId addLidar(const aslam::SensorId& base_sensor_id) {
    Lidar::UniquePtr lidar(new Lidar());
    CHECK(lidar);
    lidar->setRandom();
    CHECK(lidar->isValid());
    const aslam::SensorId& lidar_id = lidar->getId();
    aslam::Transformation T_B_S;
    T_B_S.setRandom();
    sensor_manager_.addSensor<Lidar>(std::move(lidar), base_sensor_id, T_B_S);
    return lidar_id;
  }

  aslam::SensorId addNCamera(const aslam::SensorId& base_sensor_id) {
    aslam::NCamera::UniquePtr ncamera(new aslam::NCamera);
    CHECK(ncamera);
    ncamera->setRandom();
    CHECK(ncamera->isValid());
    const aslam::SensorId& ncamera_id = ncamera->getId();
    aslam::Transformation T_B_S;
    T_B_S.setRandom();
    sensor_manager_.addSensor<aslam::NCamera>(
        std::move(ncamera), base_sensor_id, T_B_S);
    return ncamera_id;
  }

  void addVisuaInertialLidarSensorSystems() {
    // Create some test sensors.
    constexpr uint8_t kNumSensorSystems = 10u;
    constexpr uint8_t kNumNCameras = 3u;
    constexpr uint8_t kNumLidars = 2u;

    for (uint8_t sensor_idx = 0u; sensor_idx < kNumSensorSystems;
         ++sensor_idx) {
      aslam::SensorId base_sensor_id = addImu();
      for (uint8_t lidar_idx = 0u; lidar_idx < kNumLidars; ++lidar_idx) {
        addLidar(base_sensor_id);
      }
      for (uint8_t ncamera_idx = 0u; ncamera_idx < kNumNCameras;
           ++ncamera_idx) {
        addNCamera(base_sensor_id);
      }
    }
  }

 protected:
  SensorManager sensor_manager_;
};

constexpr double kPrecisionThreshold = 1e-12;

TEST_F(SensorManagerTest, YamlSerializationEmpty) {
  ASSERT_EQ(sensor_manager_.getNumSensors(), 0u);
  ASSERT_EQ(sensor_manager_.getNumSensorsOfType(SensorType::kNCamera), 0u);
  sensor_manager_.serializeToFile(
      static_cast<std::string>(serialization::internal::kYamlSensorsFilename));

  SensorManager sensor_manager_deserialized;
  ASSERT_EQ(sensor_manager_.getNumSensors(), 0u);
  ASSERT_EQ(sensor_manager_.getNumSensorsOfType(SensorType::kNCamera), 0u);
  // Expected to be equal to the empty sensor manager since no sensors added.
  EXPECT_TRUE(
      sensor_manager_.isEqual(sensor_manager_deserialized, true /*verbose*/));

  ASSERT_TRUE(sensor_manager_deserialized.deserializeFromFile(
      static_cast<std::string>(serialization::internal::kYamlSensorsFilename)));

  EXPECT_TRUE(
      sensor_manager_.isEqual(sensor_manager_deserialized, true /*verbose*/));
}

TEST_F(SensorManagerTest, YamlSerializationOneSensor) {
  addImu();

  sensor_manager_.serializeToFile(
      static_cast<std::string>(serialization::internal::kYamlSensorsFilename));

  SensorManager sensor_manager_deserialized;
  EXPECT_FALSE(
      sensor_manager_.isEqual(sensor_manager_deserialized, false /*verbose*/));

  ASSERT_TRUE(sensor_manager_deserialized.deserializeFromFile(
      static_cast<std::string>(serialization::internal::kYamlSensorsFilename)));

  EXPECT_TRUE(
      sensor_manager_.isEqual(sensor_manager_deserialized, true /*verbose*/));
}

TEST_F(SensorManagerTest, YamlSerializationOneNCamera) {
  aslam::SensorId base_id = addImu();
  addNCamera(base_id);

  sensor_manager_.serializeToFile(
      static_cast<std::string>(serialization::internal::kYamlSensorsFilename));

  SensorManager sensor_manager_deserialized;
  EXPECT_FALSE(
      sensor_manager_.isEqual(sensor_manager_deserialized, false /*verbose*/));

  ASSERT_TRUE(sensor_manager_deserialized.deserializeFromFile(
      static_cast<std::string>(serialization::internal::kYamlSensorsFilename)));

  EXPECT_TRUE(
      sensor_manager_.isEqual(sensor_manager_deserialized, true /*verbose*/));
}

TEST_F(SensorManagerTest, YamlSerializationSensorSystem) {
  addVisuaInertialLidarSensorSystems();

  sensor_manager_.serializeToFile(
      static_cast<std::string>(serialization::internal::kYamlSensorsFilename));
  SensorManager sensor_manager_deserialized;
  EXPECT_FALSE(
      sensor_manager_.isEqual(sensor_manager_deserialized, false /*verbose*/));

  ASSERT_TRUE(sensor_manager_deserialized.deserializeFromFile(
      static_cast<std::string>(serialization::internal::kYamlSensorsFilename)));
  EXPECT_TRUE(
      sensor_manager_.isEqual(sensor_manager_deserialized, true /*verbose*/));
}

}  // namespace vi_map

MAPLAB_UNITTEST_ENTRYPOINT
