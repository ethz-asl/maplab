#include <limits>
#include <string>

#include <aslam/cameras/camera-factory.h>
#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/camera.h>
#include <glog/logging.h>
#include <maplab-common/pose_types.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vi-map/vi-map.h>
#include <vi-mapping-test-app/vi-mapping-test-app.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/io/mesh_ply.h>
#include <voxblox/mesh/mesh_integrator.h>

#include "depth-integration/depth-integration.h"

const std::string kTestDataBaseFolder = "./map_resources_test_data/";  // NOLINT

const std::string kVoxbloxIntegratorType = "simple";

class VoxbloxDepthIntegrationTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    depth_map_openni_ = cv::imread(
        kTestDataBaseFolder + "/depth_map_OpenNI.pgm", CV_LOAD_IMAGE_UNCHANGED);
    CHECK_EQ(CV_MAT_TYPE(depth_map_openni_.type()), CV_16U);

    image_ = cv::imread(
        kTestDataBaseFolder + "/intensities_depth_map.pgm",
        CV_LOAD_IMAGE_GRAYSCALE);
    CHECK_EQ(CV_MAT_TYPE(image_.type()), CV_8UC1);
    CHECK_GT(image_.rows, 0);
    CHECK_GT(image_.cols, 0);

    Eigen::VectorXd intrinsics(4);
    intrinsics << 256.4159254034679, 256.3049478392699, 327.0080939570888,
        227.8040711242157;

    camera_without_distortion_ = aslam::createCamera<aslam::PinholeCamera>(
        intrinsics, image_.cols, image_.rows);
  }

  virtual void TearDown() {}

  voxblox::TsdfIntegratorBase::Config getDepthmapTsdfIntegratorConfig() const {
    voxblox::TsdfIntegratorBase::Config integrator_config;
    integrator_config.default_truncation_distance = 1.0f;
    integrator_config.max_weight = std::numeric_limits<float>::max();
    integrator_config.min_ray_length_m =
        static_cast<voxblox::FloatingPoint>(0.1);
    integrator_config.max_ray_length_m =
        static_cast<voxblox::FloatingPoint>(50.0);
    return integrator_config;
  }

  cv::Mat depth_map_openni_;
  cv::Mat image_;
  aslam::Camera::Ptr camera_without_distortion_;

  aslam::Transformation T_G_C_;
};

TEST_F(VoxbloxDepthIntegrationTest, TestIntegrateDepthMap) {
  voxblox::TsdfMap::Config tsdf_map_config;
  tsdf_map_config.tsdf_voxel_size = 0.5;
  tsdf_map_config.tsdf_voxels_per_side = 16u;
  voxblox::TsdfMap tsdf_map(tsdf_map_config);

  voxblox::TsdfIntegratorBase::Config integrator_config =
      getDepthmapTsdfIntegratorConfig();
  voxblox::TsdfIntegratorBase::Ptr integrator =
      voxblox::TsdfIntegratorFactory::create(
          kVoxbloxIntegratorType, integrator_config,
          tsdf_map.getTsdfLayerPtr());

  depth_integration::IntegrationFunctionPointCloudVoxblox integration_function =
      [&integrator](
          const voxblox::Transformation& T_G_C,
          const voxblox::Pointcloud& points, const voxblox::Colors& colors) {
        CHECK(integrator);
        integrator->integratePointCloud(T_G_C, points, colors);
      };

  vi_map::MissionId mission_id;
  aslam::generateId(&mission_id);
  depth_integration::integrateDepthMap(
      T_G_C_, 0 /*timestamp*/, mission_id, 0 /*counter*/, depth_map_openni_,
      image_, *camera_without_distortion_, integration_function);

  EXPECT_EQ(tsdf_map.getTsdfLayer().getNumberOfAllocatedBlocks(), 20u);
  EXPECT_NEAR(tsdf_map.getTsdfLayer().getMemorySize(), 984040u, 10u);

  voxblox::MeshLayer mesh_layer(tsdf_map.block_size());

  voxblox::MeshIntegratorConfig mesh_config;
  voxblox::MeshIntegrator<voxblox::TsdfVoxel> mesh_integrator(
      mesh_config, tsdf_map.getTsdfLayerPtr(), &mesh_layer);
  // We mesh the whole grid at once anyways, so all of them should be
  // updated.
  constexpr bool kMeshOnlyUpdatedBlocks = false;
  // No need to reset, we are not gonna mesh again.
  constexpr bool kResetUpdatedFlag = false;
  mesh_integrator.generateMesh(kMeshOnlyUpdatedBlocks, kResetUpdatedFlag);

  voxblox::outputMeshLayerAsPly(
      "test_results/TestIntegrateDepthMap.ply", mesh_layer);
}

MAPLAB_UNITTEST_ENTRYPOINT
