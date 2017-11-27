#include <string>

#include <gtest/gtest.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <sensors/sensor-factory.h>

#include "vi-map/sensor-manager.h"
#include "vi-map/vi-map-serialization.h"

namespace vi_map {

class SensorManagerTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    common::generateId(&mission_id_);
  }

  void addSensor() {
    Sensor::UniquePtr sensor = createRandomTestSensor();
    CHECK(sensor);
    CHECK(sensor->isValid());
    sensor_manager_.addSensor(std::move(sensor), mission_id_);
  }

  void addNCamera() {
    constexpr size_t kNumCameras = 6u;
    aslam::NCamera::Ptr ncamera =
        aslam::NCamera::createTestNCamera(kNumCameras);
    CHECK(ncamera);
    sensor_manager_.addNCamera(ncamera, mission_id_);
  }

  void addSensorSystem() {
    SensorSystem::UniquePtr sensor_system;
    // Create some test sensors.
    constexpr size_t kNumSensors = 10u;
    for (size_t sensor_idx = 0u; sensor_idx < kNumSensors; ++sensor_idx) {
      Sensor::UniquePtr sensor = createRandomTestSensor();
      CHECK(sensor);
      CHECK(sensor->isValid());

      if (sensor_idx == 0u) {
        sensor_system = aligned_unique<SensorSystem>(sensor->getId());
      } else {
        Extrinsics::UniquePtr extrinsics = Extrinsics::createRandomExtrinsics();
        CHECK(extrinsics);
        CHECK(sensor_system);
        sensor_system->setSensorExtrinsics(sensor->getId(), *extrinsics);
      }

      sensor_manager_.addSensor(std::move(sensor), mission_id_);
    }

    sensor_manager_.addSensorSystem(std::move(sensor_system));
  }

 protected:
  SensorManager sensor_manager_;
  MissionId mission_id_;
};

constexpr double kPrecisionThreshold = 1e-12;

TEST_F(SensorManagerTest, YamlSerializationEmpty) {
  sensor_manager_.serializeToFile(
      static_cast<std::string>(serialization::internal::kYamlSensorsFilename));

  SensorManager sensor_manager_deserialized;
  ASSERT_EQ(sensor_manager_deserialized.getNumSensors(), 0u);
  ASSERT_EQ(sensor_manager_deserialized.getNumNCameraSensors(), 0u);
  // Expected to be equal to the empty sensor manager since no sensors added.
  EXPECT_EQ(sensor_manager_, sensor_manager_deserialized);

  sensor_manager_deserialized.deserializeFromFile(
      static_cast<std::string>(serialization::internal::kYamlSensorsFilename));

  EXPECT_EQ(sensor_manager_, sensor_manager_deserialized);
}

TEST_F(SensorManagerTest, YamlSerializationOneSensor) {
  addSensor();

  sensor_manager_.serializeToFile(
      static_cast<std::string>(serialization::internal::kYamlSensorsFilename));

  SensorManager sensor_manager_deserialized;
  EXPECT_NE(sensor_manager_, sensor_manager_deserialized);

  sensor_manager_deserialized.deserializeFromFile(
      static_cast<std::string>(serialization::internal::kYamlSensorsFilename));

  EXPECT_EQ(sensor_manager_, sensor_manager_deserialized);
}

TEST_F(SensorManagerTest, YamlSerializationOneNCamera) {
  addNCamera();

  sensor_manager_.serializeToFile(
      static_cast<std::string>(serialization::internal::kYamlSensorsFilename));

  SensorManager sensor_manager_deserialized;
  EXPECT_NE(sensor_manager_, sensor_manager_deserialized);

  sensor_manager_deserialized.deserializeFromFile(
      static_cast<std::string>(serialization::internal::kYamlSensorsFilename));

  EXPECT_EQ(sensor_manager_, sensor_manager_deserialized);
}

TEST_F(SensorManagerTest, YamlSerializationSensorSystem) {
  addSensorSystem();

  sensor_manager_.serializeToFile(
      static_cast<std::string>(serialization::internal::kYamlSensorsFilename));
  SensorManager sensor_manager_deserialized;
  EXPECT_NE(sensor_manager_, sensor_manager_deserialized);

  sensor_manager_deserialized.deserializeFromFile(
      static_cast<std::string>(serialization::internal::kYamlSensorsFilename));
  EXPECT_EQ(sensor_manager_, sensor_manager_deserialized);
}

TEST_F(SensorManagerTest, YamlSerializationSensorsWithSensorSystems) {
  addNCamera();
  addSensorSystem();

  constexpr size_t kNumToAdd = 10u;
  for (size_t idx = 0u; idx < kNumToAdd; ++idx) {
    addSensor();
  }

  sensor_manager_.serializeToFile(
      static_cast<std::string>(serialization::internal::kYamlSensorsFilename));

  SensorManager sensor_manager_deserialized;
  EXPECT_NE(sensor_manager_, sensor_manager_deserialized);

  sensor_manager_deserialized.deserializeFromFile(
      static_cast<std::string>(serialization::internal::kYamlSensorsFilename));

  EXPECT_EQ(sensor_manager_, sensor_manager_deserialized);
}

}  // namespace vi_map

MAPLAB_UNITTEST_ENTRYPOINT
