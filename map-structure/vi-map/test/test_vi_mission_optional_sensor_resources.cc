#include <glog/logging.h>
#include <gtest/gtest.h>
#include <map-resources/optional-sensor-resources.h>
#include <maplab-common/test/testing-entrypoint.h>

#include "vi-map/sensor-manager.h"
#include "vi-map/vi-mission.h"

namespace vi_map {

class VIMissionOptionalSensorResourcesTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    sensor_id_1 = common::createRandomId<SensorId>();
    sensor_id_2 = common::createRandomId<SensorId>();

    // Add some resources for camera 1.
    mission_.addOptionalSensorResourceId(
        backend::ResourceType::kPointCloudXYZI, sensor_id_1, resource_id_1_1,
        10);
    common::generateId(&resource_id_1_2);
    mission_.addOptionalSensorResourceId(
        backend::ResourceType::kPointCloudXYZI, sensor_id_1, resource_id_1_2,
        12);
    common::generateId(&resource_id_1_3);
    mission_.addOptionalSensorResourceId(
        backend::ResourceType::kDisparityMap, sensor_id_1, resource_id_1_3, 11);

    // Add some resources for camera 2.
    common::generateId(&resource_id_2_1);
    mission_.addOptionalSensorResourceId(
        backend::ResourceType::kPointCloudXYZI, sensor_id_2, resource_id_2_1,
        10);
    common::generateId(&resource_id_2_2);
    mission_.addOptionalSensorResourceId(
        backend::ResourceType::kRawColorImage, sensor_id_2, resource_id_2_2,
        11);
    common::generateId(&resource_id_2_3);
    mission_.addOptionalSensorResourceId(
        backend::ResourceType::kDisparityMap, sensor_id_2, resource_id_2_3, 12);
  }

  SensorId sensor_id_1;
  backend::ResourceId resource_id_1_1;
  backend::ResourceId resource_id_1_2;
  backend::ResourceId resource_id_1_3;

  SensorId sensor_id_2;
  backend::ResourceId resource_id_2_1;
  backend::ResourceId resource_id_2_2;
  backend::ResourceId resource_id_2_3;

  VIMission mission_;
};

TEST_F(VIMissionOptionalSensorResourcesTest, TestGetters) {
  backend::ResourceId tmp_resource_id;

  // Check if all the resources are present and can be retrieved.
  EXPECT_TRUE(
      mission_.getOptionalSensorResourceId(
          backend::ResourceType::kPointCloudXYZI, sensor_id_1, 10,
          &tmp_resource_id));
  EXPECT_EQ(tmp_resource_id, resource_id_1_1);

  EXPECT_TRUE(
      mission_.getOptionalSensorResourceId(
          backend::ResourceType::kPointCloudXYZI, sensor_id_1, 12,
          &tmp_resource_id));
  EXPECT_EQ(tmp_resource_id, resource_id_1_2);

  EXPECT_TRUE(
      mission_.getOptionalSensorResourceId(
          backend::ResourceType::kDisparityMap, sensor_id_1, 11,
          &tmp_resource_id));
  EXPECT_EQ(tmp_resource_id, resource_id_1_3);

  EXPECT_TRUE(
      mission_.getOptionalSensorResourceId(
          backend::ResourceType::kPointCloudXYZI, sensor_id_2, 10,
          &tmp_resource_id));
  EXPECT_EQ(tmp_resource_id, resource_id_2_1);

  EXPECT_TRUE(
      mission_.getOptionalSensorResourceId(
          backend::ResourceType::kRawColorImage, sensor_id_2, 11,
          &tmp_resource_id));
  EXPECT_EQ(tmp_resource_id, resource_id_2_2);

  EXPECT_TRUE(
      mission_.getOptionalSensorResourceId(
          backend::ResourceType::kDisparityMap, sensor_id_2, 12,
          &tmp_resource_id));
  EXPECT_EQ(tmp_resource_id, resource_id_2_3);

  // Check that a wrong type, camera id and time will lead return false.
  // Wrong type.
  EXPECT_FALSE(
      mission_.getOptionalSensorResourceId(
          backend::ResourceType::kDisparityMap, sensor_id_1, 10,
          &tmp_resource_id));
  // Wrong time
  EXPECT_FALSE(
      mission_.getOptionalSensorResourceId(
          backend::ResourceType::kPointCloudXYZI, sensor_id_1, 11,
          &tmp_resource_id));
  // Wrong camera id.
  EXPECT_FALSE(
      mission_.getOptionalSensorResourceId(
          backend::ResourceType::kDisparityMap, sensor_id_2, 11,
          &tmp_resource_id));
}

TEST_F(VIMissionOptionalSensorResourcesTest, TestUniqueResourceTimestamp) {
  backend::ResourceId tmp_resource_id;
  common::generateId(&tmp_resource_id);
  EXPECT_DEATH(
      mission_.addOptionalSensorResourceId(
          backend::ResourceType::kPointCloudXYZI, sensor_id_1, tmp_resource_id,
          10),
      "Cannot store two");
}

TEST_F(VIMissionOptionalSensorResourcesTest, TestDelete) {
  backend::ResourceId tmp_resource_id;
  EXPECT_TRUE(
      mission_.getOptionalSensorResourceId(
          backend::ResourceType::kPointCloudXYZI, sensor_id_1, 10,
          &tmp_resource_id));
  EXPECT_EQ(tmp_resource_id, resource_id_1_1);

  mission_.deleteOptionalSensorResourceId(
      backend::ResourceType::kPointCloudXYZI, sensor_id_1, 10);

  EXPECT_FALSE(
      mission_.getOptionalSensorResourceId(
          backend::ResourceType::kPointCloudXYZI, sensor_id_1, 10,
          &tmp_resource_id));
}

}  // namespace vi_map

MAPLAB_UNITTEST_ENTRYPOINT
