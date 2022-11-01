#include <glog/logging.h>
#include <gtest/gtest.h>
#include <map-resources/temporal-resource-id-buffer.h>
#include <maplab-common/test/testing-entrypoint.h>

#include "vi-map/sensor-manager.h"
#include "vi-map/vi-mission.h"

namespace vi_map {

class VIMissionSensorResourcesTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    aslam::generateId(&sensor_id_1);
    aslam::generateId(&sensor_id_2);

    // Add some resources for camera 1.
    mission_.addSensorResourceId(
        backend::ResourceType::kPointCloudXYZI, sensor_id_1, resource_id_1_1,
        10);
    aslam::generateId(&resource_id_1_2);
    mission_.addSensorResourceId(
        backend::ResourceType::kPointCloudXYZI, sensor_id_1, resource_id_1_2,
        12);
    aslam::generateId(&resource_id_1_3);
    mission_.addSensorResourceId(
        backend::ResourceType::kDisparityMap, sensor_id_1, resource_id_1_3, 11);

    // Add some resources for camera 2.
    aslam::generateId(&resource_id_2_1);
    mission_.addSensorResourceId(
        backend::ResourceType::kPointCloudXYZI, sensor_id_2, resource_id_2_1,
        10);
    aslam::generateId(&resource_id_2_2);
    mission_.addSensorResourceId(
        backend::ResourceType::kRawColorImage, sensor_id_2, resource_id_2_2,
        11);
    aslam::generateId(&resource_id_2_3);
    mission_.addSensorResourceId(
        backend::ResourceType::kDisparityMap, sensor_id_2, resource_id_2_3, 12);
  }

  aslam::SensorId sensor_id_1;
  backend::ResourceId resource_id_1_1;
  backend::ResourceId resource_id_1_2;
  backend::ResourceId resource_id_1_3;

  aslam::SensorId sensor_id_2;
  backend::ResourceId resource_id_2_1;
  backend::ResourceId resource_id_2_2;
  backend::ResourceId resource_id_2_3;

  VIMission mission_;
};

TEST_F(VIMissionSensorResourcesTest, TestGetters) {
  backend::ResourceId tmp_resource_id;

  // Check if all the resources are present and can be retrieved.
  EXPECT_TRUE(mission_.getSensorResourceId(
      backend::ResourceType::kPointCloudXYZI, sensor_id_1, 10,
      &tmp_resource_id));
  EXPECT_EQ(tmp_resource_id, resource_id_1_1);

  EXPECT_TRUE(mission_.getSensorResourceId(
      backend::ResourceType::kPointCloudXYZI, sensor_id_1, 12,
      &tmp_resource_id));
  EXPECT_EQ(tmp_resource_id, resource_id_1_2);

  EXPECT_TRUE(mission_.getSensorResourceId(
      backend::ResourceType::kDisparityMap, sensor_id_1, 11, &tmp_resource_id));
  EXPECT_EQ(tmp_resource_id, resource_id_1_3);

  EXPECT_TRUE(mission_.getSensorResourceId(
      backend::ResourceType::kPointCloudXYZI, sensor_id_2, 10,
      &tmp_resource_id));
  EXPECT_EQ(tmp_resource_id, resource_id_2_1);

  EXPECT_TRUE(mission_.getSensorResourceId(
      backend::ResourceType::kRawColorImage, sensor_id_2, 11,
      &tmp_resource_id));
  EXPECT_EQ(tmp_resource_id, resource_id_2_2);

  EXPECT_TRUE(mission_.getSensorResourceId(
      backend::ResourceType::kDisparityMap, sensor_id_2, 12, &tmp_resource_id));
  EXPECT_EQ(tmp_resource_id, resource_id_2_3);

  // Check that a wrong type, camera id and time will lead return false.
  // Wrong type.
  EXPECT_FALSE(mission_.getSensorResourceId(
      backend::ResourceType::kDisparityMap, sensor_id_1, 10, &tmp_resource_id));
  // Wrong time
  EXPECT_FALSE(mission_.getSensorResourceId(
      backend::ResourceType::kPointCloudXYZI, sensor_id_1, 11,
      &tmp_resource_id));
  // Wrong camera id.
  EXPECT_FALSE(mission_.getSensorResourceId(
      backend::ResourceType::kDisparityMap, sensor_id_2, 11, &tmp_resource_id));
}

TEST_F(VIMissionSensorResourcesTest, TestUniqueResourceTimestamp) {
  backend::ResourceId tmp_resource_id;
  aslam::generateId(&tmp_resource_id);
  EXPECT_DEATH(
      mission_.addSensorResourceId(
          backend::ResourceType::kPointCloudXYZI, sensor_id_1, tmp_resource_id,
          10),
      "Cannot store two");
}

TEST_F(VIMissionSensorResourcesTest, TestDelete) {
  backend::ResourceId tmp_resource_id;
  EXPECT_TRUE(mission_.getSensorResourceId(
      backend::ResourceType::kPointCloudXYZI, sensor_id_1, 10,
      &tmp_resource_id));
  EXPECT_EQ(tmp_resource_id, resource_id_1_1);

  mission_.deleteSensorResourceId(
      backend::ResourceType::kPointCloudXYZI, sensor_id_1, 10);

  EXPECT_FALSE(mission_.getSensorResourceId(
      backend::ResourceType::kPointCloudXYZI, sensor_id_1, 10,
      &tmp_resource_id));
}

}  // namespace vi_map

MAPLAB_UNITTEST_ENTRYPOINT
