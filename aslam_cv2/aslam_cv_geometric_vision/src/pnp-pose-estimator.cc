#include <memory>

#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/camera-unified-projection.h>
#include <aslam/common/memory.h>
#include <opengv/absolute_pose/CentralAbsoluteAdapter.hpp>
#include <opengv/absolute_pose/NoncentralAbsoluteAdapter.hpp>
#include <opengv/absolute_pose/methods.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp>

#include "aslam/geometric-vision/pnp-pose-estimator.h"

namespace aslam {
namespace geometric_vision {

bool PnpPoseEstimator::absolutePoseRansacPinholeCam(
    const Eigen::Matrix2Xd& measurements,
    const Eigen::Matrix3Xd& G_landmark_positions, double pixel_sigma,
    int max_ransac_iters, aslam::Camera::ConstPtr camera_ptr,
    aslam::Transformation* T_G_C, std::vector<int>* inliers, int* num_iters) const {
  CHECK_NOTNULL(T_G_C);
  CHECK_NOTNULL(inliers);
  CHECK_NOTNULL(num_iters);
  CHECK_EQ(measurements.cols(), G_landmark_positions.cols());

  double ransac_threshold = 1;
  using aslam::PinholeCamera;
  using aslam::UnifiedProjectionCamera;
  switch (camera_ptr->getType()) {
    case aslam::Camera::Type::kPinhole: {
      const double fu =
          camera_ptr->getParameters()(PinholeCamera::Parameters::kFu);
      const double fv =
          camera_ptr->getParameters()(PinholeCamera::Parameters::kFv);

      const double focal_length = (fu + fv) / 2.0;
      ransac_threshold = 1.0 - cos(atan(pixel_sigma / focal_length));
      break;
    }
    case aslam::Camera::Type::kUnifiedProjection: {
      const double fu =
          camera_ptr->getParameters()(UnifiedProjectionCamera::Parameters::kFu);
      const double fv =
          camera_ptr->getParameters()(UnifiedProjectionCamera::Parameters::kFv);

      const double focal_length = (fu + fv) / 2.0;
      ransac_threshold = 1.0 - cos(atan(pixel_sigma / focal_length));
      break;
    }
    default:
      LOG(FATAL) << "Unknown camera type. The given camera is neither of type "
                 << "Pinhole nor UnifiedProjection.";
  }

  // Assuming the mean of lens focal lengths is the best estimate here.
  return absolutePoseRansac(
      measurements, G_landmark_positions, ransac_threshold, max_ransac_iters,
      camera_ptr, T_G_C, inliers, num_iters);
}

bool PnpPoseEstimator::absoluteMultiPoseRansacPinholeCam(
    const Eigen::Matrix2Xd& measurements,
    const std::vector<int>& measurement_camera_indices,
    const Eigen::Matrix3Xd& G_landmark_positions, double pixel_sigma,
    int max_ransac_iters, aslam::NCamera::ConstPtr ncamera_ptr,
    aslam::Transformation* T_G_I, std::vector<int>* inliers, int* num_iters) const {
  std::vector<double> inlier_distances_to_model;
  return absoluteMultiPoseRansacPinholeCam(
      measurements, measurement_camera_indices, G_landmark_positions,
      pixel_sigma, max_ransac_iters, ncamera_ptr, T_G_I, inliers,
      &inlier_distances_to_model, num_iters);
}

bool PnpPoseEstimator::absoluteMultiPoseRansacPinholeCam(
    const Eigen::Matrix2Xd& measurements,
    const std::vector<int>& measurement_camera_indices,
    const Eigen::Matrix3Xd& G_landmark_positions, double pixel_sigma,
    int max_ransac_iters, aslam::NCamera::ConstPtr ncamera_ptr,
    aslam::Transformation* T_G_I, std::vector<int>* inliers,
    std::vector<double>* inlier_distances_to_model, int* num_iters) const {
  CHECK_NOTNULL(T_G_I);
  CHECK_NOTNULL(inliers);
  CHECK_NOTNULL(inlier_distances_to_model);
  CHECK_NOTNULL(num_iters);
  CHECK_EQ(measurements.cols(), G_landmark_positions.cols());
  CHECK_EQ(
      measurements.cols(),
      static_cast<int>(measurement_camera_indices.size()));

  const size_t num_cameras = ncamera_ptr->getNumCameras();
  double focal_length = 0;

  // Average focal length together over all cameras in both axes.
  using aslam::PinholeCamera;
  using aslam::UnifiedProjectionCamera;
  for (size_t camera_index = 0; camera_index < num_cameras; ++camera_index) {
    const aslam::Camera::ConstPtr& camera_ptr =
        ncamera_ptr->getCameraShared(camera_index);
    switch (camera_ptr->getType()) {
      case aslam::Camera::Type::kPinhole: {
        const double fu =
            camera_ptr->getParameters()(PinholeCamera::Parameters::kFu);
        const double fv =
            camera_ptr->getParameters()(PinholeCamera::Parameters::kFv);
        focal_length += (fu + fv);
        break;
      }
      case aslam::Camera::Type::kUnifiedProjection: {
        const double fu = camera_ptr->getParameters()(
            UnifiedProjectionCamera::Parameters::kFu);
        const double fv = camera_ptr->getParameters()(
            UnifiedProjectionCamera::Parameters::kFv);

        focal_length += (fu + fv);
        break;
      }
      default:
        LOG(FATAL) << "Unknown camera type.  The given camera is neither of "
                   << "type Pinhole nor UnifiedProjection.";
    }
  }

  focal_length /= (2.0 * static_cast<double>(num_cameras));

  const double ransac_threshold = 1.0 - cos(atan(pixel_sigma / focal_length));

  return absoluteMultiPoseRansac(
      measurements, measurement_camera_indices, G_landmark_positions,
      ransac_threshold, max_ransac_iters, ncamera_ptr, T_G_I, inliers,
      inlier_distances_to_model, num_iters);
}

bool PnpPoseEstimator::absolutePoseRansac(
    const Eigen::Matrix2Xd& measurements,
    const Eigen::Matrix3Xd& G_landmark_positions, double ransac_threshold,
    int max_ransac_iters, aslam::Camera::ConstPtr camera_ptr,
    aslam::Transformation* T_G_C, std::vector<int>* inliers, int* num_iters) const {
  CHECK_NOTNULL(T_G_C);
  CHECK_NOTNULL(inliers);
  CHECK_NOTNULL(num_iters);
  CHECK_EQ(measurements.cols(), G_landmark_positions.cols());

  opengv::points_t points;
  opengv::bearingVectors_t bearing_vectors;
  points.resize(measurements.cols());
  bearing_vectors.resize(measurements.cols());
  for (int i = 0; i < measurements.cols(); ++i) {
    camera_ptr->backProject3(measurements.col(i), &bearing_vectors[i]);
    bearing_vectors[i].normalize();
    points[i] = G_landmark_positions.col(i);
  }

  opengv::absolute_pose::CentralAbsoluteAdapter adapter(
      bearing_vectors, points);
  opengv::sac::Ransac<
      opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem>
      ransac;
  std::shared_ptr<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem>
      absposeproblem_ptr(
          new opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem(
              adapter, opengv::sac_problems::absolute_pose::
                  AbsolutePoseSacProblem::KNEIP, random_seed_));
  ransac.sac_model_ = absposeproblem_ptr;
  ransac.threshold_ = ransac_threshold;
  ransac.max_iterations_ = max_ransac_iters;
  bool ransac_success = ransac.computeModel();

  if (ransac_success) {
    T_G_C->getPosition() = ransac.model_coefficients_.rightCols(1);
    Eigen::Matrix<double, 3, 3> R_G_C(ransac.model_coefficients_.leftCols(3));
    T_G_C->getRotation() = aslam::Quaternion(R_G_C);
  }

  *inliers = ransac.inliers_;
  *num_iters = ransac.iterations_;
  return ransac_success;
}

bool PnpPoseEstimator::absoluteMultiPoseRansac(
    const Eigen::Matrix2Xd& measurements,
    const std::vector<int>& measurement_camera_indices,
    const Eigen::Matrix3Xd& G_landmark_positions, double ransac_threshold,
    int max_ransac_iters, aslam::NCamera::ConstPtr ncamera_ptr,
    aslam::Transformation* T_G_I, std::vector<int>* inliers, int* num_iters) const {
  std::vector<double> inlier_distances_to_model;
  return absoluteMultiPoseRansac(
      measurements, measurement_camera_indices, G_landmark_positions,
      ransac_threshold, max_ransac_iters, ncamera_ptr, T_G_I, inliers,
      &inlier_distances_to_model, num_iters);
}

bool PnpPoseEstimator::absoluteMultiPoseRansac(
    const Eigen::Matrix2Xd& measurements,
    const std::vector<int>& measurement_camera_indices,
    const Eigen::Matrix3Xd& G_landmark_positions, double ransac_threshold,
    int max_ransac_iters, aslam::NCamera::ConstPtr ncamera_ptr,
    aslam::Transformation* T_G_I, std::vector<int>* inliers,
    std::vector<double>* inlier_distances_to_model, int* num_iters) const {
  CHECK_NOTNULL(T_G_I);
  CHECK_NOTNULL(inliers);
  CHECK_NOTNULL(inlier_distances_to_model);
  CHECK_NOTNULL(num_iters);
  CHECK_EQ(measurements.cols(), G_landmark_positions.cols());
  CHECK_EQ(
      measurements.cols(),
      static_cast<int>(measurement_camera_indices.size()));

  // Fill in camera information from NCamera.
  // Rotation matrix for each camera.
  opengv::rotations_t cam_rotations;
  opengv::translations_t cam_translations;

  const int num_cameras = ncamera_ptr->getNumCameras();

  cam_rotations.resize(num_cameras);
  cam_translations.resize(num_cameras);

  for (int camera_index = 0; camera_index < num_cameras; ++camera_index) {
    const aslam::Transformation& T_C_B = ncamera_ptr->get_T_C_B(camera_index);
    // OpenGV requires body frame -> camera transformation.
    aslam::Transformation T_B_C = T_C_B.inverse();
    cam_rotations[camera_index] = T_B_C.getRotationMatrix();
    cam_translations[camera_index] = T_B_C.getPosition();
  }

  opengv::points_t points;
  opengv::bearingVectors_t bearing_vectors;
  points.resize(measurements.cols());
  bearing_vectors.resize(measurements.cols());
  for (int i = 0; i < measurements.cols(); ++i) {
    // Figure out which camera this corresponds to, and reproject it in the
    // correct camera.
    int camera_index = measurement_camera_indices[i];
    ncamera_ptr->getCamera(camera_index)
        .backProject3(measurements.col(i), &bearing_vectors[i]);
    bearing_vectors[i].normalize();
    points[i] = G_landmark_positions.col(i);
  }
  // Basically same as the Central, except measurement_camera_indices, which
  // assigns a camera index to each bearing_vector, and cam_offsets and
  // cam_rotations, which describe the position and orientation of the cameras
  // with respect to the body frame.
  opengv::absolute_pose::NoncentralAbsoluteAdapter adapter(
      bearing_vectors, measurement_camera_indices, points, cam_translations,
      cam_rotations);
  opengv::sac::Ransac<
      opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem>
      ransac;
  std::shared_ptr<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem>
      absposeproblem_ptr(
          new opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem(
              adapter, opengv::sac_problems::absolute_pose::
                AbsolutePoseSacProblem::GP3P, random_seed_));
  ransac.sac_model_ = absposeproblem_ptr;
  ransac.threshold_ = ransac_threshold;
  ransac.max_iterations_ = max_ransac_iters;
  bool ransac_success = ransac.computeModel();
  CHECK_EQ(ransac.inliers_.size(), ransac.inlier_distances_to_model_.size());

  if (ransac_success) {
    // Optional nonlinear model refinement over all inliers.
    Eigen::Matrix<double, 3, 4> final_model = ransac.model_coefficients_;
    if (run_nonlinear_refinement_) {
      absposeproblem_ptr->optimizeModelCoefficients(
          ransac.inliers_, ransac.model_coefficients_, final_model);
    }

    // Set result.
    T_G_I->getPosition() = final_model.rightCols(1);
    Eigen::Matrix<double, 3, 3> R_G_I(final_model.leftCols(3));
    T_G_I->getRotation() = aslam::Quaternion(R_G_I);
  }

  *inliers = ransac.inliers_;
  *inlier_distances_to_model = ransac.inlier_distances_to_model_;
  *num_iters = ransac.iterations_;

  return ransac_success;
}

bool PnpPoseEstimator::absoluteMultiPoseRansac3DFeatures(
    const Eigen::Matrix3Xd& measurements,
    const std::vector<int>& measurement_camera_indices,
    const Eigen::Matrix3Xd& G_landmark_positions, const double ransac_threshold,
    const int max_ransac_iters, const double pnp_3d_ransac_stopping_ratio,
    aslam::NCamera::ConstPtr ncamera_ptr, aslam::Transformation* T_G_I,
    std::vector<int>* inliers, std::vector<double>* inlier_distances_to_model,
    int* num_iters) const {
  CHECK_NOTNULL(ncamera_ptr);
  CHECK_NOTNULL(T_G_I);
  CHECK_NOTNULL(inliers)->clear();
  CHECK_NOTNULL(inlier_distances_to_model)->clear();
  CHECK_NOTNULL(num_iters);

  const std::size_t n_measurements = measurements.cols();
  CHECK_EQ(n_measurements, G_landmark_positions.cols())
      << "Measurements and landmarks are not in pairs.";
  if (measurements.size() < 6) {
    return false;
  }

  const aslam::Transformation& T_C_B = ncamera_ptr->get_T_C_B(0);
  const aslam::Transformation T_B_C = T_C_B.inverse();
  const Eigen::Matrix3d& R_B_C = T_B_C.getRotationMatrix();
  const Eigen::Vector3d& p_B_C = T_B_C.getPosition();

  std::vector<Eigen::Vector3d> landmarks;
  std::vector<Eigen::Vector3d> observations;

  for (std::size_t i = 0u; i < n_measurements; ++i) {
    landmarks.emplace_back(G_landmark_positions.col(i));
    observations.emplace_back(R_B_C * measurements.col(i) + p_B_C);
  }
  Eigen::Matrix3d ransac_rotation_matrix;
  Eigen::Vector3d ransac_translation;
  std::vector<std::size_t> ransac_outliers;

  ransacTransformationFor3DPoints(
      landmarks, observations, ransac_threshold, max_ransac_iters,pnp_3d_ransac_stopping_ratio,
      &ransac_rotation_matrix, &ransac_translation, inliers, &ransac_outliers);

  // Set result.
  T_G_I->getPosition() = ransac_translation;
  T_G_I->getRotationMatrix() = ransac_rotation_matrix;
  for (const int& inlier : *inliers) {
    inlier_distances_to_model->emplace_back(
        (ransac_rotation_matrix * landmarks[inlier] + ransac_translation -
         observations[inlier])
            .norm());
  }
  *num_iters = max_ransac_iters;
  return true;
}

void PnpPoseEstimator::ransacTransformationFor3DPoints(
    const std::vector<Eigen::Vector3d>& point_set_1,
    const std::vector<Eigen::Vector3d>& point_set_2, const double ransac_threshold,
    const std::size_t ransac_max_iterations,
    const double pnp_3d_ransac_stopping_ratio,
    Eigen::Matrix3d* best_rotation_matrix, Eigen::Vector3d* best_translation,
    std::vector<int>* best_inliers, std::vector<size_t>* best_outliers) const {
  CHECK_NOTNULL(best_rotation_matrix);
  CHECK_NOTNULL(best_translation);
  CHECK_NOTNULL(best_inliers);
  CHECK_NOTNULL(best_outliers);
  CHECK_GT(ransac_threshold, 0.0);
  CHECK_GT(ransac_max_iterations, 0u);

  const std::size_t n_point_set_1 = point_set_1.size();
  const std::size_t n_point_set_2 = point_set_2.size();
  CHECK_EQ(n_point_set_1, n_point_set_2);
  CHECK_GT(n_point_set_2, 6u);

  std::vector<std::size_t> outliers;
  for (std::size_t j = 0u; j < ransac_max_iterations; ++j) {
    // Generate 6 unique random indices for Ransac.
    if (!random_seed_) {
      std::srand(time(0));
    }
    std::vector<std::size_t> ransac_indices(n_point_set_1);
    std::iota(ransac_indices.begin(), ransac_indices.end(), 0);
    std::random_shuffle(ransac_indices.begin(), ransac_indices.end());

    // Generate transformation matrix.
    // https://igl.ethz.ch/projects/ARAP/svd_rot.pdf
    Eigen::Matrix3d X, Y;
    X << point_set_1[ransac_indices[0]], point_set_1[ransac_indices[1]],
        point_set_1[ransac_indices[2]], point_set_1[ransac_indices[3]],
        point_set_1[ransac_indices[4]], point_set_1[ransac_indices[5]];
    Y << point_set_2[ransac_indices[0]], point_set_2[ransac_indices[1]],
        point_set_2[ransac_indices[2]], point_set_2[ransac_indices[3]],
        point_set_2[ransac_indices[4]], point_set_2[ransac_indices[5]];

    const Eigen::VectorXd X_mean = X.rowwise().mean();
    const Eigen::VectorXd Y_mean = Y.rowwise().mean();

    X.colwise() -= X_mean;
    Y.colwise() -= Y_mean;

    const Eigen::Matrix3d S = X * Y.transpose();
    const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        S, Eigen::ComputeThinU | Eigen::ComputeThinV);

    Eigen::Matrix3d reflection_handler = Eigen::Matrix3d::Identity();
    reflection_handler(2, 2) =
        (svd.matrixV() * svd.matrixU().transpose()).determinant();
    const Eigen::Matrix3d R =
        svd.matrixV() * reflection_handler * svd.matrixU().transpose();
    const Eigen::Vector3d t = Y_mean - R * X_mean;

    // Calculate Outliers for Transformation
    outliers.clear();
    best_inliers->clear();

    for (std::size_t i = 0u; i < n_point_set_1; ++i) {
      const Eigen::Vector3d transformation_error =
          R * point_set_1[i] + t - point_set_2[i];
      if (transformation_error.norm() / point_set_2[i].norm() >
          ransac_threshold) {
        outliers.emplace_back(i);
      } else {
        best_inliers->emplace_back(i);
      }
    }
    // Compare results
    const std::size_t n_inliers = best_inliers->size();
    if (n_inliers > best_inliers->size()) {
      *best_outliers = outliers;
      *best_rotation_matrix = R;
      *best_translation = t;
    }
    const double inlier_ratio = n_inliers / (n_inliers + outliers.size());
    if (inlier_ratio > pnp_3d_ransac_stopping_ratio) {
      break;
    }
  }
}

}  // namespace geometric_vision
}  // namespace aslam
