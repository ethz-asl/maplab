#include "geometric-vision/five-point-pose-estimator.h"

#include <memory>
#include <vector>

#include <aslam/common/memory.h>
#include <maplab-common/quaternion-math.h>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>

namespace opengv_pose_estimation {

void FivePointPoseEstimator::ComputePinhole(
    const Eigen::Matrix2Xd& measurements_a,
    const Eigen::Matrix2Xd& measurements_b, double pixel_sigma,
    unsigned int max_ransac_iters, aslam::Camera::ConstPtr camera_ptr,
    pose::Transformation* output_transform, std::vector<int>* inlier_matches) {
  CHECK_NOTNULL(output_transform);
  CHECK_NOTNULL(inlier_matches);
  CHECK_EQ(measurements_a.cols(), measurements_b.cols());

  // This method is designed for pinhole camera, so we should be able to cast
  // the base camera ptr to the derived pinhole type.
  aslam::PinholeCamera::ConstPtr pinhole_camera_ptr =
      std::static_pointer_cast<const aslam::PinholeCamera>(camera_ptr);
  CHECK(pinhole_camera_ptr)
      << "Couldn't cast camera pointer to pinhole camera type.";

  // Assuming the mean of lens focal lengths is the best estimate here.
  const double focal_length =
      (pinhole_camera_ptr->fu() + pinhole_camera_ptr->fv()) / 2.0;
  const double ransac_threshold = 1.0 - cos(atan(pixel_sigma / focal_length));

  Compute(
      measurements_a, measurements_b, ransac_threshold, max_ransac_iters,
      camera_ptr, output_transform, inlier_matches);
}

void FivePointPoseEstimator::Compute(
    const Eigen::Matrix2Xd& measurements_a,
    const Eigen::Matrix2Xd& measurements_b, double ransac_threshold,
    unsigned int max_ransac_iters, aslam::Camera::ConstPtr camera_ptr,
    pose::Transformation* output_transform, std::vector<int>* inlier_matches) {
  CHECK_NOTNULL(output_transform);
  CHECK_NOTNULL(inlier_matches);
  CHECK_EQ(measurements_a.cols(), measurements_b.cols());
  CHECK_GE(measurements_a.cols(), 5);

  opengv::bearingVectors_t bearing_vectors_a;
  opengv::bearingVectors_t bearing_vectors_b;
  bearing_vectors_a.resize(measurements_a.cols());
  bearing_vectors_b.resize(measurements_b.cols());

  for (unsigned int i = 0; i < measurements_a.cols(); ++i) {
    camera_ptr->backProject3(measurements_a.col(i), &bearing_vectors_a[i]);
    bearing_vectors_a[i].normalize();
    camera_ptr->backProject3(measurements_b.col(i), &bearing_vectors_b[i]);
    bearing_vectors_b[i].normalize();
  }

  opengv::rotation_t rotation;
  rotation.setIdentity();

  // create a central relative adapter
  opengv::relative_pose::CentralRelativeAdapter adapter(
      bearing_vectors_a, bearing_vectors_b, rotation);

  opengv::sac::Ransac<
      opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem>
      ransac;
  std::shared_ptr<
      opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem>
      relposeproblem_ptr(
          new opengv::sac_problems::relative_pose::
              CentralRelativePoseSacProblem(
                  adapter, opengv::sac_problems::relative_pose::
                               CentralRelativePoseSacProblem::NISTER));
  ransac.sac_model_ = relposeproblem_ptr;
  ransac.threshold_ = ransac_threshold;
  ransac.max_iterations_ = max_ransac_iters;

  ransac.computeModel();

  inlier_matches->swap(ransac.inliers_);

  output_transform->getPosition() = ransac.model_coefficients_.rightCols(1);
  const Eigen::Matrix3d A_R_B = ransac.model_coefficients_.leftCols(3);
  output_transform->getRotation().toImplementation() =
      Eigen::Quaterniond(A_R_B).normalized();
}

}  // namespace opengv_pose_estimation
