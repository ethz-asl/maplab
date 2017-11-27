#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/distortion-radtan.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <map-resources/optional-sensor-resources.h>
#include <maplab-common/test/testing-entrypoint.h>

#include "vi-map/vi-mission.h"

namespace vi_map {

class VIMissionOptionalCameraResourcesTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    aslam::Camera::ConstPtr camera_1 =
        aslam::PinholeCamera::createTestCamera<aslam::RadTanDistortion>();
    aslam::Camera::ConstPtr camera_2 =
        aslam::PinholeCamera::createTestCamera<aslam::RadTanDistortion>();
    camera_id_1 = camera_1->getId();
    camera_id_2 = camera_2->getId();
    aslam::Transformation T_C1_B;
    T_C1_B.setRandom();
    aslam::Transformation T_C2_B;
    T_C2_B.setRandom();
    mission_.addOptionalCameraWithExtrinsics(*camera_1, T_C1_B);
    mission_.addOptionalCameraWithExtrinsics(*camera_2, T_C2_B);

    // Add some resources for camera 1.
    common::generateId(&resource_id_1_1);
    mission_.addOptionalCameraResourceId(
        backend::ResourceType::kRawImage, camera_id_1, resource_id_1_1, 10);
    common::generateId(&resource_id_1_2);
    mission_.addOptionalCameraResourceId(
        backend::ResourceType::kRawImage, camera_id_1, resource_id_1_2, 12);
    common::generateId(&resource_id_1_3);
    mission_.addOptionalCameraResourceId(
        backend::ResourceType::kDisparityMap, camera_id_1, resource_id_1_3, 11);

    // Add some resources for camera 2.
    common::generateId(&resource_id_2_1);
    mission_.addOptionalCameraResourceId(
        backend::ResourceType::kRawImage, camera_id_2, resource_id_2_1, 10);
    common::generateId(&resource_id_2_2);
    mission_.addOptionalCameraResourceId(
        backend::ResourceType::kRawColorImage, camera_id_2, resource_id_2_2,
        11);
    common::generateId(&resource_id_2_3);
    mission_.addOptionalCameraResourceId(
        backend::ResourceType::kDisparityMap, camera_id_2, resource_id_2_3, 12);
  }

  aslam::CameraId camera_id_1;
  backend::ResourceId resource_id_1_1;
  backend::ResourceId resource_id_1_2;
  backend::ResourceId resource_id_1_3;

  aslam::CameraId camera_id_2;
  backend::ResourceId resource_id_2_1;
  backend::ResourceId resource_id_2_2;
  backend::ResourceId resource_id_2_3;

  VIMission mission_;
};

TEST_F(VIMissionOptionalCameraResourcesTest, TestGetters) {
  backend::ResourceId tmp_resource_id;

  // Check if all the resources are present and can be retrieved.
  EXPECT_TRUE(
      mission_.getOptionalCameraResourceId(
          backend::ResourceType::kRawImage, camera_id_1, 10, &tmp_resource_id));
  EXPECT_EQ(tmp_resource_id, resource_id_1_1);

  EXPECT_TRUE(
      mission_.getOptionalCameraResourceId(
          backend::ResourceType::kRawImage, camera_id_1, 12, &tmp_resource_id));
  EXPECT_EQ(tmp_resource_id, resource_id_1_2);

  EXPECT_TRUE(
      mission_.getOptionalCameraResourceId(
          backend::ResourceType::kDisparityMap, camera_id_1, 11,
          &tmp_resource_id));
  EXPECT_EQ(tmp_resource_id, resource_id_1_3);

  EXPECT_TRUE(
      mission_.getOptionalCameraResourceId(
          backend::ResourceType::kRawImage, camera_id_2, 10, &tmp_resource_id));
  EXPECT_EQ(tmp_resource_id, resource_id_2_1);

  EXPECT_TRUE(
      mission_.getOptionalCameraResourceId(
          backend::ResourceType::kRawColorImage, camera_id_2, 11,
          &tmp_resource_id));
  EXPECT_EQ(tmp_resource_id, resource_id_2_2);

  EXPECT_TRUE(
      mission_.getOptionalCameraResourceId(
          backend::ResourceType::kDisparityMap, camera_id_2, 12,
          &tmp_resource_id));
  EXPECT_EQ(tmp_resource_id, resource_id_2_3);

  // Check that a wrong type, camera id and time will lead return false.
  // Wrong type.
  EXPECT_FALSE(
      mission_.getOptionalCameraResourceId(
          backend::ResourceType::kDisparityMap, camera_id_1, 10,
          &tmp_resource_id));
  // Wrong time
  EXPECT_FALSE(
      mission_.getOptionalCameraResourceId(
          backend::ResourceType::kRawImage, camera_id_1, 11, &tmp_resource_id));
  // Wrong camera id.
  EXPECT_FALSE(
      mission_.getOptionalCameraResourceId(
          backend::ResourceType::kDisparityMap, camera_id_2, 11,
          &tmp_resource_id));
}

TEST_F(VIMissionOptionalCameraResourcesTest, TestUniqueResourceTimestamp) {
  backend::ResourceId tmp_resource_id;
  common::generateId(&tmp_resource_id);
  EXPECT_DEATH(
      mission_.addOptionalCameraResourceId(
          backend::ResourceType::kRawImage, camera_id_1, tmp_resource_id, 10),
      "Cannot store two");
}

TEST_F(VIMissionOptionalCameraResourcesTest, TestDelete) {
  backend::ResourceId tmp_resource_id;
  EXPECT_TRUE(
      mission_.getOptionalCameraResourceId(
          backend::ResourceType::kRawImage, camera_id_1, 10, &tmp_resource_id));
  EXPECT_EQ(tmp_resource_id, resource_id_1_1);

  mission_.deleteOptionalCameraResourceId(
      backend::ResourceType::kRawImage, camera_id_1, 10);

  EXPECT_FALSE(
      mission_.getOptionalCameraResourceId(
          backend::ResourceType::kRawImage, camera_id_1, 10, &tmp_resource_id));
}

}  // namespace vi_map

MAPLAB_UNITTEST_ENTRYPOINT
