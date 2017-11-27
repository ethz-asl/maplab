#include "geometric-vision/relative-non-central-pnp.h"

#include <memory>
#include <vector>

#include <aslam/cameras/camera-pinhole.h>
#include <maplab-common/quaternion-math.h>
#include <opengv/relative_pose/NoncentralRelativeMultiAdapter.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <opengv/sac/MultiRansac.hpp>
#include <opengv/sac_problems/relative_pose/MultiNoncentralRelativePoseSacProblem.hpp>

namespace geometric_vision {

bool RelativeNonCentralPnp::computePinhole(
    const aslam::VisualNFrame::ConstPtr& nframe_0,
    const aslam::VisualNFrame::ConstPtr& nframe_1,
    const aslam::FrameToFrameMatchesList& matches_0_1, double pixel_sigma,
    size_t ransac_max_iters, pose::Transformation* T_I0_I1,
    size_t* num_inliers) const {
  CHECK_GT(pixel_sigma, 0.0);
  CHECK_NOTNULL(T_I0_I1);
  CHECK_NOTNULL(num_inliers);

  const aslam::NCamera::ConstPtr& ncamera_ptr = nframe_0->getNCameraShared();
  const size_t num_cameras = ncamera_ptr->getNumCameras();
  // Ensuring that the camera_rig does not change from viewpoint 0 to viewpoint
  // 1.
  CHECK_EQ(
      nframe_0->getNCameraShared().get(), nframe_1->getNCameraShared().get());

  // Check minimum number of matches.
  size_t num_matches = 0;
  for (size_t camera_idx = 0; camera_idx < num_cameras; ++camera_idx) {
    num_matches += matches_0_1[camera_idx].size();
  }
  if (static_cast<size_t>(getMinRequiredMatches()) > num_matches) {
    LOG(FATAL) << "Not enough matches.";
    T_I0_I1->setIdentity();
    return false;
  }

  double focal_length = 0;
  // Average focal length together over all cameras in both axes.
  // TODO(helenol): Perhaps this is a reasonable thing to do for multiple
  // cameras?
  for (size_t camera_index = 0; camera_index < num_cameras; ++camera_index) {
    aslam::PinholeCamera::ConstPtr pinhole_camera_ptr =
        std::dynamic_pointer_cast<const aslam::PinholeCamera>(
            ncamera_ptr->getCameraShared(camera_index));
    CHECK(pinhole_camera_ptr)
        << "Couldn't cast camera pointer to pinhole camera type.";
    focal_length += pinhole_camera_ptr->fu() + pinhole_camera_ptr->fv();
  }
  focal_length /= 2.0 * num_cameras;
  const double ransac_threshold = 1.0 - cos(atan(pixel_sigma / focal_length));

  bool success = compute(
      nframe_0, nframe_1, matches_0_1, ransac_threshold, ransac_max_iters,
      T_I0_I1, num_inliers);

  double inlier_ratio =
      static_cast<double>(*num_inliers) / static_cast<double>(num_matches);
  VLOG(10) << "Relative pose estimator ransac inlier ratio: " << inlier_ratio
           << " (" << *num_inliers << " / " << num_matches << ")";

  // The ransac solution is assumed to be valid if the inlier ratio is above a
  // threshold or
  // the absolute number of inliers is about 2-3 times the min. required
  // matches.
  const size_t kRansacInlierCountThreshold = 2 * getMinRequiredMatches();
  if (inlier_ratio < kRansacInlierRatioThreshold &&
      *num_inliers < kRansacInlierCountThreshold) {
    T_I0_I1->setIdentity();
    LOG(WARNING)
        << "Relative pose RANSAC failed: inlier ratio below threshold of "
        << kRansacInlierRatioThreshold << " with " << *num_inliers << " out of "
        << num_matches << ".";
    success = false;
  }
  return success;
}

bool RelativeNonCentralPnp::compute(
    const aslam::VisualNFrame::ConstPtr& nframe_0,
    const aslam::VisualNFrame::ConstPtr& nframe_1,
    const aslam::FrameToFrameMatchesList& matches_0_1, double ransac_threshold,
    size_t ransac_max_iters, pose::Transformation* T_I0_I1,
    size_t* num_inliers) const {
  std::vector<std::shared_ptr<opengv::bearingVectors_t> >
      multi_bearing_vectors_0;
  std::vector<std::shared_ptr<opengv::bearingVectors_t> >
      multi_bearing_vectors_1;
  // Iterate through the cameras (pairs).
  const aslam::NCamera::ConstPtr& ncamera_ptr = nframe_0->getNCameraShared();
  CHECK_EQ(
      nframe_0->getNCameraShared().get(), nframe_1->getNCameraShared().get());

  const size_t num_cameras = ncamera_ptr->getNumCameras();
  CHECK_EQ(matches_0_1.size(), num_cameras);
  for (size_t camera_idx = 0; camera_idx < num_cameras; ++camera_idx) {
    // Create the bearing-vector arrays for this camera.
    std::shared_ptr<opengv::bearingVectors_t> bearing_vectors_0(
        new opengv::bearingVectors_t);
    std::shared_ptr<opengv::bearingVectors_t> bearing_vectors_1(
        new opengv::bearingVectors_t);

    // Now iterate through the observations of that camera.
    // Cache current camera ...
    const aslam::Camera& current_camera = ncamera_ptr->getCamera(camera_idx);
    // ... and the observations made from this camera
    // in viewpoint 0 and 1.
    const Eigen::Matrix2Xd& measurements_0 =
        nframe_0->getFrame(camera_idx).getKeypointMeasurements();
    const Eigen::Matrix2Xd& measurements_1 =
        nframe_1->getFrame(camera_idx).getKeypointMeasurements();

    // Fix index of first frame...
    for (size_t i = 0; i < matches_0_1[camera_idx].size(); ++i) {
      size_t keypoint_idx_nframe_0 = matches_0_1[camera_idx][i].first;
      size_t keypoint_idx_nframe_1 = matches_0_1[camera_idx][i].second;
      CHECK_LT(static_cast<int>(keypoint_idx_nframe_0), measurements_0.cols());
      CHECK_LT(static_cast<int>(keypoint_idx_nframe_1), measurements_1.cols());

      // Unproject the keypoint pairs and grab the bearing vector ...
      opengv::bearingVector_t bearing_vector_0, bearing_vector_1;

      // ... for viewpoint 0 ...
      current_camera.backProject3(
          measurements_0.col(keypoint_idx_nframe_0), &bearing_vector_0);
      // ... and viewpoint 1.
      current_camera.backProject3(
          measurements_1.col(keypoint_idx_nframe_1), &bearing_vector_1);

      bearing_vectors_0->emplace_back(bearing_vector_0.normalized());
      bearing_vectors_1->emplace_back(bearing_vector_1.normalized());
    }
    // Push back all bearing vectors for both nframes for this camera.
    multi_bearing_vectors_0.push_back(bearing_vectors_0);
    multi_bearing_vectors_1.push_back(bearing_vectors_1);
  }

  // Transform camera_rig geometry to openGV types.
  opengv::rotations_t camera_rotations;
  opengv::translations_t camera_offsets;
  camera_rotations.resize(num_cameras);
  camera_offsets.resize(num_cameras);

  for (size_t camera_idx = 0; camera_idx < num_cameras; ++camera_idx) {
    const pose::Transformation& T_C_B = ncamera_ptr->get_T_C_B(camera_idx);
    // OpenGV requires T_B_C transformation.
    pose::Transformation T_B_C = T_C_B.inverse();
    camera_rotations[camera_idx] = T_B_C.getRotationMatrix();
    camera_offsets[camera_idx] = T_B_C.getPosition();
  }

  // Create a non-central relative multi-adapter.
  opengv::relative_pose::NoncentralRelativeMultiAdapter adapter(
      multi_bearing_vectors_0, multi_bearing_vectors_1, camera_offsets,
      camera_rotations);

  // Create a MultiNoncentralRelativePoseSacProblem and Ransac.
  opengv::sac::MultiRansac<MultiNonCentralRelativeSacPnp> ransac(random_seed_);

  std::shared_ptr<MultiNonCentralRelativeSacPnp> problem_ptr(
      new MultiNonCentralRelativeSacPnp(
          adapter, solver_, kAsCentral, random_seed_));

  ransac.sac_model_ = problem_ptr;
  ransac.threshold_ = ransac_threshold;
  ransac.max_iterations_ = ransac_max_iters;

  // Run RANSAC.
  int verbose_level = static_cast<int>(VLOG_IS_ON(10));
  bool success = ransac.computeModel(verbose_level);

  if (success) {
    // Optional nonlinear model refinement over all inliers.
    Eigen::Matrix<double, 3, 4> final_model = ransac.model_coefficients_;
    if (run_nonlinear_refinement_) {
      problem_ptr->optimizeModelCoefficients(
          ransac.inliers_, ransac.model_coefficients_, final_model);
    }

    // Calculate the number of RANSAC inliers.
    size_t number_inliers_counter = 0;
    for (size_t i = 0; i < ransac.inliers_.size(); ++i) {
      number_inliers_counter += ransac.inliers_[i].size();
    }

    // Assign the output values.
    *num_inliers = number_inliers_counter;
    T_I0_I1->getRotation() =
        pose::Quaternion(static_cast<Eigen::Matrix3d>(final_model.leftCols(3)));
    T_I0_I1->getPosition() = final_model.rightCols(1);
  } else {
    *num_inliers = 0;
    T_I0_I1->setIdentity();
  }
  return success;
}
}  // namespace geometric_vision
