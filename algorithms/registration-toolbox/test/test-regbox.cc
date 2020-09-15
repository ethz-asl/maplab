#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <pcl/point_types.h>
#include "registration-toolbox/common/base-controller.h"
#include "registration-toolbox/mock-controller.h"
#include "registration-toolbox/pcl-gicp-controller.h"
#include "registration-toolbox/pcl-icp-controller.h"

namespace regbox {

class RegBoxTest : public ::testing::Test {
 protected:
  RegBoxTest() : ::testing::Test() {}

  virtual void SetUp() {
    constexpr uint32_t width = 50;
    target_ = generateRandomCloud(width);
    source_ = generateRandomCloud(width);
    CHECK_NOTNULL(target_);
    CHECK_NOTNULL(source_);

    LOG(INFO) << "Generate target and source cloud with " << target_->size()
              << " and " << source_->size() << " points.";
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr generateRandomCloud(
      const uint32_t width) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZI>);
    cloud->width = width;
    cloud->height = 1;
    cloud->is_dense = false;

    const uint32_t n_points = cloud->width * cloud->height;
    cloud->points.resize(n_points);
    // TODO(lbern): change to a modern random number generator.
    srand(time(NULL));
    uint32_t seed = time(NULL);
    for (uint32_t i = 0u; i < n_points; ++i) {
      cloud->points[i].x = 1024 * rand_r(&seed) / (RAND_MAX + 1.0f);
      cloud->points[i].y = 1024 * rand_r(&seed) / (RAND_MAX + 1.0f);
      cloud->points[i].z = 1024 * rand_r(&seed) / (RAND_MAX + 1.0f);
      cloud->points[i].intensity = 1024 * rand_r(&seed) / (RAND_MAX + 1.0f);
    }
    return cloud;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr target_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr source_;
};

TEST_F(RegBoxTest, TestControllerCreationIcpString) {
  auto aligner =
      regbox::BaseController::make("regbox::PclIcpController", "PCL ICP");
  EXPECT_NE(nullptr, aligner);
}

TEST_F(RegBoxTest, TestControllerCreationIcpEnum) {
  auto aligner = regbox::BaseController::make(Aligner::PclIcp, "PCL ICP");
  EXPECT_NE(nullptr, aligner);
}

TEST_F(RegBoxTest, TestControllerCreationGIcpString) {
  auto aligner = regbox::BaseController::make(
      "regbox::PclGeneralizedIcpController", "PCL GICP");
  EXPECT_NE(nullptr, aligner);
}

TEST_F(RegBoxTest, TestControllerCreationGIcpEnum) {
  auto aligner = regbox::BaseController::make(Aligner::PclGIcp, "PCL GICP");
  EXPECT_NE(nullptr, aligner);
}

TEST_F(RegBoxTest, TestControllerCreationMockString) {
  auto aligner = regbox::BaseController::make("regbox::MockController", "Mock");
  EXPECT_NE(nullptr, aligner);
}

TEST_F(RegBoxTest, TestControllerCreationMockEnum) {
  auto aligner = regbox::BaseController::make(Aligner::PclGIcp, "Mock");
  EXPECT_NE(nullptr, aligner);
}

TEST_F(RegBoxTest, TestControllerCreationLbmIcpString) {
  auto aligner = regbox::BaseController::make(
      "regbox::LpmIcpController", "Libpointmatcher ICP");
  EXPECT_NE(nullptr, aligner);
}

TEST_F(RegBoxTest, TestControllerCreationLbmIcpEnum) {
  auto aligner =
      regbox::BaseController::make(Aligner::LpmIcp, "Libpointmatcher ICP");
  EXPECT_NE(nullptr, aligner);
}

TEST_F(RegBoxTest, TestControllerCreationFailure) {
  auto aligner = regbox::BaseController::make("foo", "PCL ICP");
  EXPECT_EQ(nullptr, aligner);
}

TEST_F(RegBoxTest, TestControllerSanityRegistrationICP) {
  auto aligner = regbox::BaseController::make(Aligner::PclIcp, "PCL ICP");
  EXPECT_NE(nullptr, aligner);

  aslam::Transformation prior;
  prior.setIdentity();
  regbox::RegistrationResult result = aligner->align(target_, source_, prior);
  EXPECT_EQ(true, result.hasConverged());
}

TEST_F(RegBoxTest, TestControllerSanityRegistrationGICP) {
  auto aligner = regbox::BaseController::make(Aligner::PclGIcp, "PCL GICP");
  EXPECT_NE(nullptr, aligner);

  aslam::Transformation prior;
  prior.setIdentity();
  regbox::RegistrationResult result = aligner->align(target_, source_, prior);
  EXPECT_EQ(true, result.hasConverged());
}

TEST_F(RegBoxTest, TestControllerSanityRegistrationMock) {
  auto aligner = regbox::BaseController::make(Aligner::Mock, "Mock");
  EXPECT_NE(nullptr, aligner);

  aslam::Transformation prior;
  prior.setIdentity();
  regbox::RegistrationResult result = aligner->align(target_, source_, prior);
  EXPECT_EQ(true, result.hasConverged());
}

}  // namespace regbox

MAPLAB_UNITTEST_ENTRYPOINT
