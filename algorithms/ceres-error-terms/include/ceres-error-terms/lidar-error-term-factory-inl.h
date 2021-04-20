#ifndef CERES_ERROR_TERMS_LIDAR_ERROR_TERM_FACTORY_INL_H_
#define CERES_ERROR_TERMS_LIDAR_ERROR_TERM_FACTORY_INL_H_

#include <limits>
#include <vector>

#include <glog/logging.h>

#include <aslam/cameras/camera-3d-lidar.h>
#include <aslam/cameras/camera.h>

#include "ceres-error-terms/common.h"

namespace ceres_error_terms {

template <template <typename, typename> class ErrorTerm>
ceres::CostFunction* createLidarCostFunction(
    const Eigen::Vector3d& measurement, double pixel_sigma,
    ceres_error_terms::visual::VisualErrorType error_term_type,
    aslam::Camera* camera) {
  CHECK_NOTNULL(camera);
  ceres::CostFunction* error_term = nullptr;
  if (camera->getType() != aslam::Camera::Type::kLidar3D) {
    LOG(FATAL) << "Wrong Camera type for this term";
  }
  aslam::Camera3DLidar* derived_camera =
      dynamic_cast<aslam::Camera3DLidar*>(camera);
  CHECK_NOTNULL(derived_camera);

  error_term = new ErrorTerm<aslam::Camera3DLidar, aslam::NullDistortion>(
      measurement, pixel_sigma, error_term_type, derived_camera);
  return error_term;
}

// TODO(mariusbr) This will need some adjustments to the measurements maybe
void replaceUnusedArgumentsOfLidarCostFunctionWithDummies(
    ceres_error_terms::visual::VisualErrorType error_term_type,
    std::vector<double*>* error_term_argument_list,
    std::vector<double*>* dummies_to_set_constant) {
  CHECK_NOTNULL(error_term_argument_list);
  CHECK_NOTNULL(dummies_to_set_constant)->clear();

  CHECK_EQ(error_term_argument_list->size(), 8u);
  for (const double* argument : *error_term_argument_list) {
    CHECK_NOTNULL(argument);
  }

  // Initialize dummy variables to infinity so that any usage mistakes can be
  // detected.
  static Eigen::Matrix<double, 7, 1> dummy_7d_landmark_base_pose =
      Eigen::Matrix<double, 7, 1>::Constant(std::numeric_limits<double>::max());
  static Eigen::Matrix<double, 7, 1> dummy_7d_landmark_mission_base_pose =
      Eigen::Matrix<double, 7, 1>::Constant(std::numeric_limits<double>::max());
  static Eigen::Matrix<double, 7, 1> dummy_7d_imu_mission_base_pose =
      Eigen::Matrix<double, 7, 1>::Constant(std::numeric_limits<double>::max());
  static Eigen::Matrix<double, 7, 1> dummy_7d_imu_pose =
      Eigen::Matrix<double, 7, 1>::Constant(std::numeric_limits<double>::max());

  if (error_term_type == visual::VisualErrorType::kLocalKeyframe) {
    // The baseframes and keyframe poses are not necessary in the local
    // mission case.
    (*error_term_argument_list)[1] = dummy_7d_landmark_base_pose.data();
    (*error_term_argument_list)[2] = dummy_7d_landmark_mission_base_pose.data();
    (*error_term_argument_list)[3] = dummy_7d_imu_mission_base_pose.data();
    (*error_term_argument_list)[4] = dummy_7d_imu_pose.data();
    dummies_to_set_constant->emplace_back(dummy_7d_landmark_base_pose.data());
    dummies_to_set_constant->emplace_back(
        dummy_7d_landmark_mission_base_pose.data());
    dummies_to_set_constant->emplace_back(
        dummy_7d_imu_mission_base_pose.data());
    dummies_to_set_constant->emplace_back(dummy_7d_imu_pose.data());
  } else if (error_term_type == visual::VisualErrorType::kLocalMission) {
    // The baseframes are not necessary in the local mission case.
    (*error_term_argument_list)[2] = dummy_7d_landmark_base_pose.data();
    (*error_term_argument_list)[3] = dummy_7d_landmark_mission_base_pose.data();
    dummies_to_set_constant->emplace_back(dummy_7d_landmark_base_pose.data());
    dummies_to_set_constant->emplace_back(
        dummy_7d_landmark_mission_base_pose.data());
  } else if (error_term_type == visual::VisualErrorType::kGlobal) {
    // Nothing to replace.
  } else {
    LOG(FATAL) << "Unknown error term type: " << error_term_type;
  }
}

}  // namespace ceres_error_terms

#endif  // CERES_ERROR_TERMS_LIDAR_ERROR_TERM_FACTORY_INL_H_
