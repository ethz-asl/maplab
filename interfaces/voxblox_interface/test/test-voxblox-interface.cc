#include <limits>
#include <string>

#include <aslam/cameras/camera-factory.h>
#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/camera.h>
#include <glog/logging.h>
#include <landmark-triangulation/landmark-triangulation.h>
#include <map-manager/map-manager.h>
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

#include "voxblox-interface/integration.h"

const std::string kTestDataBaseFolder = "./map_resources_test_data/"; // NOLINT

class VoxbloxInterfaceTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    vi_map::VIMapManager map_manager;
    CHECK(
        map_manager.loadMapFromFolder("./test_maps/vi_app_test", &vi_map_key_));
    CHECK(
        landmark_triangulation::retriangulateLandmarks(
            map_manager.getMapMutable(vi_map_key_)));

    depth_map_openni_ = cv::imread(
        kTestDataBaseFolder + "/depth_map_OpenNI.pgm", CV_LOAD_IMAGE_UNCHANGED);
    CHECK_EQ(CV_MAT_TYPE(depth_map_openni_.type()), CV_16U);

    image_ = cv::imread(
        kTestDataBaseFolder + "/intensities_OpenNI.pgm",
        CV_LOAD_IMAGE_GRAYSCALE);
    CHECK_EQ(CV_MAT_TYPE(image_.type()), CV_8UC1);

    Eigen::VectorXd intrinsics(4);
    intrinsics << 256.4159254034679, 256.3049478392699, 327.0080939570888,
        227.8040711242157;

    camera_without_distortion_ = aslam::createCamera<aslam::PinholeCamera>(
        intrinsics, image_.cols, image_.rows);
  }

  virtual void TearDown() {
    vi_map::VIMapManager map_manager;
    if (map_manager.hasMap(vi_map_key_)) {
      map_manager.deleteMap(vi_map_key_);
    }
  }

  voxblox::TsdfIntegratorBase::Config getLandmarksTsdfIntegratorConfig() const {
    voxblox::TsdfIntegratorBase::Config integrator_config;
    integrator_config.default_truncation_distance = 0.1f;
    integrator_config.max_weight = std::numeric_limits<float>::max();
    integrator_config.min_ray_length_m =
        static_cast<voxblox::FloatingPoint>(0.1);
    integrator_config.max_ray_length_m =
        static_cast<voxblox::FloatingPoint>(5.0);
    integrator_config.use_sparsity_compensation_factor = true;
    integrator_config.sparsity_compensation_factor = 30.f;
    return integrator_config;
  }

  voxblox::TsdfIntegratorBase::Config getDepthmapTsdfIntegratorConfig() const {
    voxblox::TsdfIntegratorBase::Config integrator_config;
    integrator_config.default_truncation_distance = 1.0f;
    integrator_config.max_weight = std::numeric_limits<float>::max();
    integrator_config.min_ray_length_m =
        static_cast<voxblox::FloatingPoint>(0.1);
    integrator_config.max_ray_length_m =
        static_cast<voxblox::FloatingPoint>(50.0);
    integrator_config.use_sparsity_compensation_factor = false;
    integrator_config.sparsity_compensation_factor = 30.f;
    return integrator_config;
  }

  const vi_map::VIMap& getViMap() const {
    vi_map::VIMapManager map_manager;
    CHECK(map_manager.hasMap(vi_map_key_));
    return map_manager.getMap(vi_map_key_);
  }

  cv::Mat depth_map_openni_;
  cv::Mat image_;
  aslam::Camera::Ptr camera_without_distortion_;

  pose::Transformation T_G_C_;

 private:
  std::string vi_map_key_;
};

TEST_F(VoxbloxInterfaceTest, TestIntegrateAllLandmarks) {
  voxblox::TsdfMap::Config tsdf_map_config;
  tsdf_map_config.tsdf_voxel_size = 0.25;
  tsdf_map_config.tsdf_voxels_per_side = 8u;
  voxblox::TsdfMap tsdf_map(tsdf_map_config);
  voxblox::TsdfIntegratorBase::Config integrator_config =
      getLandmarksTsdfIntegratorConfig();

  voxblox_interface::integrateAllLandmarks(
      getViMap(), integrator_config, &tsdf_map);

  EXPECT_EQ(tsdf_map.getTsdfLayer().getNumberOfAllocatedBlocks(), 45u);
  EXPECT_NEAR(tsdf_map.getTsdfLayer().getMemorySize(), 279482u, 10u);
  // TODO(fabianbl): Compare to serialized TSDF map.
}

TEST_F(VoxbloxInterfaceTest, TestIntegrateDepthMap) {
  voxblox::TsdfMap::Config tsdf_map_config;
  tsdf_map_config.tsdf_voxel_size = 0.5;
  tsdf_map_config.tsdf_voxels_per_side = 16u;
  voxblox::TsdfMap tsdf_map(tsdf_map_config);
  voxblox::TsdfIntegratorBase::Config integrator_config =
      getDepthmapTsdfIntegratorConfig();

  voxblox::SimpleTsdfIntegrator tsdf_integrator(
      integrator_config, tsdf_map.getTsdfLayerPtr());

  voxblox_interface::integrateDepthMap(
      T_G_C_, depth_map_openni_, image_, *camera_without_distortion_,
      &tsdf_integrator);

  EXPECT_EQ(tsdf_map.getTsdfLayer().getNumberOfAllocatedBlocks(), 20u);
  EXPECT_NEAR(tsdf_map.getTsdfLayer().getMemorySize(), 984392u, 10u);

  voxblox::MeshLayer mesh_layer(tsdf_map.block_size());

  voxblox::MeshIntegrator<voxblox::TsdfVoxel>::Config mesh_config;
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
