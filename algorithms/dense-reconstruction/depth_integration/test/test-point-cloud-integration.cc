#include <limits>
#include <string>

#include <glog/logging.h>
#include <map-manager/map-manager.h>
#include <maplab-common/pose_types.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <vi-map/vi-map.h>
#include <vi-mapping-test-app/vi-mapping-test-app.h>

#include "depth-integration/depth-integration.h"

const std::string kTestDataBaseFolder = "./map_resources_test_data/";  // NOLINT

const std::string kVoxbloxIntegratorType = "simple";

class PointCloudIntegrationTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    vi_map::VIMapManager map_manager;
    CHECK(map_manager.loadMapFromFolder(
        "./test_maps/dense_mapping_test_map", &vi_map_key_));
  }

  virtual void TearDown() {
    vi_map::VIMapManager map_manager;
    if (map_manager.hasMap(vi_map_key_)) {
      map_manager.deleteMap(vi_map_key_);
    }
  }

  const vi_map::VIMap& getViMap() const {
    vi_map::VIMapManager map_manager;
    CHECK(map_manager.hasMap(vi_map_key_));
    return map_manager.getMap(vi_map_key_);
  }

 private:
  std::string vi_map_key_;
};

TEST_F(PointCloudIntegrationTest, TestIntegrationFunctionPointCloudVoxblox) {
  size_t counter = 0u;

  depth_integration::IntegrationFunctionPointCloudVoxblox integration_function =
      [&counter](
          const vi_map::MissionId& /*mission_id*/,
          const int64_t /*timestamp_ns*/, const aslam::SensorId& /*sensor_id*/,
          const voxblox::Transformation& /*T_G_S*/,
          const voxblox::Pointcloud& points, const voxblox::Colors& colors) {
        ASSERT_FALSE(points.empty());
        ASSERT_FALSE(colors.empty());
        ++counter;
      };

  const vi_map::VIMap& vi_map = getViMap();
  vi_map::MissionIdList mission_ids;
  vi_map.getAllMissionIds(&mission_ids);
  depth_integration::integrateAllDepthResourcesOfType(
      mission_ids, backend::ResourceType::kPointCloudXYZI,
      false /*use_undistorted_camera_for_depth_maps*/, vi_map,
      integration_function, nullptr /*resource_selection_function*/);

  EXPECT_EQ(counter, 24u);
}

TEST_F(PointCloudIntegrationTest, TestIntegrationFunctionPointCloudMaplab) {
  size_t counter = 0u;

  depth_integration::IntegrationFunctionPointCloudMaplab integration_function =
      [&counter](
          const vi_map::MissionId& /*mission_id*/,
          const int64_t /*timestamp_ns*/, const aslam::SensorId& /*sensor_id*/,
          const aslam::Transformation& /*T_G_S*/,
          const resources::PointCloud& points_C) {
        ASSERT_FALSE(points_C.empty());

        ASSERT_FALSE(points_C.hasColor());
        ASSERT_FALSE(points_C.hasNormals());
        ASSERT_FALSE(points_C.hasLabels());
        ASSERT_TRUE(points_C.hasScalars());

        ++counter;
      };

  const vi_map::VIMap& vi_map = getViMap();
  vi_map::MissionIdList mission_ids;
  vi_map.getAllMissionIds(&mission_ids);
  depth_integration::integrateAllDepthResourcesOfType(
      mission_ids, backend::ResourceType::kPointCloudXYZI,
      false /*use_undistorted_camera_for_depth_maps*/, vi_map,
      integration_function, nullptr /*resource_selection_function*/);

  EXPECT_EQ(counter, 24u);
}

TEST_F(
    PointCloudIntegrationTest,
    TestIntegrationFunctionPointCloudMaplabSelection) {
  size_t counter = 0u;
  size_t selection_counter = 0u;

  depth_integration::IntegrationFunctionPointCloudMaplab integration_function =
      [&counter](
          const vi_map::MissionId& /*mission_id*/,
          const int64_t /*timestamp_ns*/, const aslam::SensorId& /*sensor_id*/,
          const aslam::Transformation& /*T_G_S*/,
          const resources::PointCloud& points_C) {
        ASSERT_FALSE(points_C.empty());

        ASSERT_FALSE(points_C.hasColor());
        ASSERT_FALSE(points_C.hasNormals());
        ASSERT_FALSE(points_C.hasLabels());
        ASSERT_TRUE(points_C.hasScalars());

        ++counter;
      };

  // Select all.
  depth_integration::ResourceSelectionFunction one_function_to_select_them_all =
      [&selection_counter](
          const int64_t /*timestamp_ns*/,
          const aslam::Transformation& /*T_G_S*/) {
        ++selection_counter;
        return true;
      };

  const vi_map::VIMap& vi_map = getViMap();
  vi_map::MissionIdList mission_ids;
  vi_map.getAllMissionIds(&mission_ids);
  depth_integration::integrateAllDepthResourcesOfType(
      mission_ids, backend::ResourceType::kPointCloudXYZI,
      false /*use_undistorted_camera_for_depth_maps*/, vi_map,
      integration_function, one_function_to_select_them_all);

  EXPECT_EQ(counter, 24u);
  EXPECT_EQ(selection_counter, 24u);

  // Reset counter.
  counter = 0u;
  selection_counter = 0u;

  // Select none.
  depth_integration::ResourceSelectionFunction one_function_to_shun_them_all =
      [&selection_counter](
          const int64_t /*timestamp_ns*/,
          const aslam::Transformation& /*T_G_S*/) {
        ++selection_counter;
        return false;
      };

  depth_integration::integrateAllDepthResourcesOfType(
      mission_ids, backend::ResourceType::kPointCloudXYZI,
      false /*use_undistorted_camera_for_depth_maps*/, vi_map,
      integration_function, one_function_to_shun_them_all);

  EXPECT_EQ(counter, 0u);
  EXPECT_EQ(selection_counter, 24u);

  // Reset counter.
  counter = 0u;
  selection_counter = 0u;

  aslam::Transformation::Position G_p_center;
  G_p_center << 48.4692, 18.9221, 0.504537;
  const double radius_m = 10.0;

  // Special integration function to check whether selection function is doing
  // its job.
  depth_integration::IntegrationFunctionPointCloudMaplab
      selective_integration_function =
          [&counter, &G_p_center, &radius_m](
              const vi_map::MissionId& /*mission_id*/,
              const int64_t /*timestamp_ns*/,
              const aslam::SensorId& /*sensor_id*/,
              const aslam::Transformation& T_G_S,
              const resources::PointCloud& points_C) {
            ASSERT_FALSE(points_C.empty());

            ASSERT_FALSE(points_C.hasColor());
            ASSERT_FALSE(points_C.hasNormals());
            ASSERT_FALSE(points_C.hasLabels());
            ASSERT_TRUE(points_C.hasScalars());

            ASSERT_TRUE((T_G_S.getPosition() - G_p_center).norm() < radius_m);

            ++counter;
          };

  // Select within a radius.
  depth_integration::ResourceSelectionFunction
      one_function_to_find_the_chosen_ones =
          [&selection_counter, &G_p_center, &radius_m](
              const int64_t /*timestamp_ns*/,
              const aslam::Transformation& T_G_S) -> bool {
    ++selection_counter;
    return (T_G_S.getPosition() - G_p_center).norm() < radius_m;
  };

  depth_integration::integrateAllDepthResourcesOfType(
      mission_ids, backend::ResourceType::kPointCloudXYZI,
      false /*use_undistorted_camera_for_depth_maps*/, vi_map,
      selective_integration_function, one_function_to_find_the_chosen_ones);

  EXPECT_EQ(counter, 9u);
  EXPECT_EQ(selection_counter, 24u);
}

TEST_F(PointCloudIntegrationTest, TestIntegrationFunctionDepthImage) {
  // This resource type is not supported by the depth map integration function,
  // so the integration function should never be called!

  depth_integration::IntegrationFunctionDepthImage integration_function =
      [](const vi_map::MissionId& /*mission_id*/,
         const int64_t /*timestamp_ns*/, const aslam::Transformation& /*T_G_C*/,
         const aslam::Camera& /*camera*/, const cv::Mat& /*depth_image*/,
         const cv::Mat& /*intensity_image*/) { ASSERT_TRUE(false); };

  const vi_map::VIMap& vi_map = getViMap();
  vi_map::MissionIdList mission_ids;
  vi_map.getAllMissionIds(&mission_ids);
  depth_integration::integrateAllDepthResourcesOfType(
      mission_ids, backend::ResourceType::kPointCloudXYZI,
      false /*use_undistorted_camera_for_depth_maps*/, vi_map,
      integration_function, nullptr /*resource_selection_function*/);
}

MAPLAB_UNITTEST_ENTRYPOINT
