#ifndef CERES_ERROR_TERMS_LANDMARK_COMMON_INL_H_
#define CERES_ERROR_TERMS_LANDMARK_COMMON_INL_H_

#include <limits>
#include <vector>

namespace ceres_error_terms {

void replaceUnusedArgumentsOfLandmarkCostFunctionWithDummies(
    ceres_error_terms::LandmarkErrorType error_term_type,
    std::vector<double*>* error_term_argument_list,
    std::vector<double*>* dummies_to_set_constant) {
  CHECK_NOTNULL(error_term_argument_list);
  CHECK_NOTNULL(dummies_to_set_constant)->clear();

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

  if (error_term_type == LandmarkErrorType::kLocalKeyframe) {
    // The baseframes and keyframe poses are not necessary in the local
    // mission case.
    (*error_term_argument_list)[1] = dummy_7d_landmark_base_pose.data();
    (*error_term_argument_list)[2] = dummy_7d_landmark_mission_base_pose.data();
    (*error_term_argument_list)[3] = dummy_7d_imu_mission_base_pose.data();
    (*error_term_argument_list)[4] = dummy_7d_imu_pose.data();
    dummies_to_set_constant->push_back(dummy_7d_landmark_base_pose.data());
    dummies_to_set_constant->push_back(
        dummy_7d_landmark_mission_base_pose.data());
    dummies_to_set_constant->push_back(dummy_7d_imu_mission_base_pose.data());
    dummies_to_set_constant->push_back(dummy_7d_imu_pose.data());
  } else if (error_term_type == LandmarkErrorType::kLocalMission) {
    // The baseframes are not necessary in the local mission case.
    (*error_term_argument_list)[2] = dummy_7d_landmark_base_pose.data();
    (*error_term_argument_list)[3] = dummy_7d_landmark_mission_base_pose.data();
    dummies_to_set_constant->push_back(dummy_7d_landmark_base_pose.data());
    dummies_to_set_constant->push_back(
        dummy_7d_landmark_mission_base_pose.data());
  } else if (error_term_type == LandmarkErrorType::kGlobal) {
    // Nothing to replace.
  } else {
    LOG(FATAL) << "Unknown error term type: " << error_term_type;
  }
}

}  // namespace ceres_error_terms

#endif  // CERES_ERROR_TERMS_LANDMARK_COMMON_INL_H_
