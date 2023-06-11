#include <Eigen/Core>
#include <Eigen/StdVector>
#include <eigen-checks/gtest.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <aslam/calibration/focallength-initializers.h>
#include <aslam/calibration/target-aprilgrid.h>
#include <aslam/calibration/target-observation.h>
#include <aslam/cameras/camera.h>
#include <aslam/cameras/camera-factory.h>
#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/distortion.h>
#include <aslam/common/entrypoint.h>
#include <aslam/common/pose-types.h>

const double kDegToRad = M_PI / 180.0;

///////////////////////////////////////////////
// Types to test
///////////////////////////////////////////////
template<typename Camera, typename Distortion>
struct CameraDistortion {
  typedef Camera CameraType;
  typedef Distortion DistortionType;
};

using testing::Types;
typedef Types<
    //CameraDistortion<aslam::PinholeCamera,aslam::FisheyeDistortion>,
    CameraDistortion<aslam::PinholeCamera, aslam::EquidistantDistortion>,
    CameraDistortion<aslam::PinholeCamera, aslam::RadTanDistortion>,
    CameraDistortion<aslam::PinholeCamera,aslam::NullDistortion>
    //CameraDistortion<aslam::UnifiedProjectionCamera,aslam::FisheyeDistortion>,
    //CameraDistortion<aslam::UnifiedProjectionCamera,aslam::EquidistantDistortion>,
    //CameraDistortion<aslam::UnifiedProjectionCamera,aslam::RadTanDistortion>,
    //CameraDistortion<aslam::UnifiedProjectionCamera,aslam::NullDistortion>
    >
    Implementations;

///////////////////////////////////////////////
// Test fixture
///////////////////////////////////////////////
template <typename CameraDistortion>
class TestCameras : public testing::Test {
 public:
  typedef typename CameraDistortion::CameraType CameraType;
  typedef typename CameraDistortion::DistortionType DistortionType;
 protected:
  TestCameras() : camera_(CameraType::template createTestCamera<DistortionType>()) {
    // Make sure the focal length are the same in both directions, as the
    // initializers assume that.
    CHECK(camera_);
    Eigen::VectorXd intrinsics = camera_->getParameters();
    intrinsics[aslam::PinholeCamera::kFu] =
        intrinsics[aslam::PinholeCamera::kFv];
    camera_->setParameters(intrinsics);
  };
  virtual ~TestCameras() {};

  bool runInitialization(
      const std::vector<aslam::calibration::TargetObservation::Ptr>& observations,
      Eigen::VectorXd* intrinsics_vector) const {
    CHECK_NOTNULL(intrinsics_vector);

    return aslam::calibration::FocalLengthInitializer<CameraType, DistortionType>::initialize(
        observations, intrinsics_vector);
  }

  typename CameraType::Ptr camera_;
};

TYPED_TEST_CASE(TestCameras, Implementations);

aslam::calibration::TargetObservation::Ptr simulateTargetObservation(
    const aslam::calibration::TargetAprilGrid::Ptr& target,
    const aslam::Camera& camera, const aslam::Transformation& T_C_T) {
  CHECK(target);

  // Project the target points into the camera frame and then onto the image plane/
  const Eigen::Matrix3Xd& points_target_frame = target->points();
  const Eigen::Matrix3Xd points_camera_frame = T_C_T.transformVectorized(points_target_frame);

  Eigen::Matrix2Xd image_points;
  std::vector<aslam::ProjectionResult> results;
  camera.project3Vectorized(points_camera_frame, &image_points, &results);

  // Remove corners that were projected outside the image boundary.
  Eigen::Matrix2Xd visible_image_points(2, target->size());
  Eigen::VectorXi visible_corner_ids(target->size());
  size_t num_visible_corners = 0u;

  for (size_t corner_idx = 0u; corner_idx < results.size(); ++corner_idx) {
    if (!results[corner_idx].isKeypointVisible()) {
      continue;
    }

    visible_image_points.col(num_visible_corners) = image_points.col(corner_idx);
    visible_corner_ids(num_visible_corners) = corner_idx;
    ++num_visible_corners;
  }
  visible_image_points.conservativeResize(Eigen::NoChange, num_visible_corners);
  visible_corner_ids.conservativeResize(num_visible_corners);

  return aslam::calibration::TargetObservation::Ptr(
      new aslam::calibration::TargetObservation(target, camera.imageHeight(),
                                                camera.imageWidth(), visible_corner_ids,
                                                visible_image_points));
}

// Tests are temporally disabled due to possible malfunction. Further checks required.
TYPED_TEST(TestCameras, DISABLED_InitializeIntrinsics) {
  CHECK(this->camera_);

  // Create a target April grid.
  aslam::calibration::TargetAprilGrid::TargetConfiguration aprilgrid_config;
  aslam::calibration::TargetAprilGrid::Ptr aprilgrid(
      new aslam::calibration::TargetAprilGrid(aprilgrid_config));

  // Simulate poses and reproject the target.
  constexpr size_t kNumCamPoses = 50;
  constexpr double kMaxTranslationM = 1.0;
  constexpr double kMaxRotationDeg = 60.0;

  aslam::Transformation T_C_T_nominal;
  T_C_T_nominal.getPosition() = 1.5 * Eigen::Vector3d::UnitZ();

  std::vector<aslam::calibration::TargetObservation::Ptr> target_observations;
  while (target_observations.size() < kNumCamPoses) {
    // Add a random disturbance around the nominal point from which the target is
    // completely visible.
    aslam::Transformation disturbance;
    disturbance.setRandom(kMaxTranslationM, kMaxRotationDeg * kDegToRad);
    aslam::Transformation T_C_T = T_C_T_nominal * disturbance;

    aslam::calibration::TargetObservation::Ptr obs =
        simulateTargetObservation(aprilgrid, *this->camera_, T_C_T);

    // Skip incomplete simulated targets.
    if (!obs->allCornersObservered()) {
      continue;
    }
    target_observations.push_back(obs);
  }

  // Run the initialization over all targets.
  Eigen::VectorXd intrinsics_guess(this->camera_->getParameterSize());
  bool intrinsics_success = this->runInitialization(target_observations, &intrinsics_guess);
  EXPECT_TRUE(intrinsics_success) << "Intrinsics initialization failed.";

  const double kTolerance = 1.0;
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(intrinsics_guess, this->camera_->getParameters(), kTolerance));
}

ASLAM_UNITTEST_ENTRYPOINT
