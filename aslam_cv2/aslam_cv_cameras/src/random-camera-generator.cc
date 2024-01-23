#include "aslam/cameras/random-camera-generator.h"

#include "aslam/cameras/distortion-equidistant.h"
#include "aslam/cameras/distortion-fisheye.h"
#include "aslam/cameras/distortion-null.h"
#include "aslam/cameras/distortion-radtan.h"
#include "aslam/cameras/distortion.h"

namespace aslam {

NCamera::Ptr createTestNCamera(size_t num_cameras) {
  return aligned_shared<aslam::NCamera>(*createUniqueTestNCamera(num_cameras));
}

NCamera::UniquePtr createUniqueTestNCamera(size_t num_cameras) {
  std::vector<Camera::Ptr> cameras;
  Aligned<std::vector, Transformation> T_C_B_vector;

  for (size_t camera_idx = 0u; camera_idx < num_cameras; ++camera_idx) {
    cameras.emplace_back(PinholeCamera::createTestCamera<RadTanDistortion>());

    // Offset each camera 0.1 m in x direction and rotate it to face forward.
    Eigen::Vector3d position(0.1 * (camera_idx + 1), 0.0, 0.0);
    Quaternion q_C_B(0.5, 0.5, -0.5, 0.5);
    Transformation T_C_B(q_C_B, position);
    T_C_B_vector.emplace_back(T_C_B);
  }

  NCameraId rig_id;
  generateId(&rig_id);
  TransformationCovariance T_G_B_fixed_localization_covariance =
      TransformationCovariance::Identity();
  std::string description("Test camera rig");
  return std::move(aligned_unique<aslam::NCamera>(
      rig_id, T_C_B_vector, T_G_B_fixed_localization_covariance, cameras,
      description));
}

NCamera::Ptr createSurroundViewTestNCamera() {
  return aligned_shared<aslam::NCamera>(*createSurroundViewUniqueTestNCamera());
}

NCamera::UniquePtr createSurroundViewUniqueTestNCamera() {
  std::vector<Camera::Ptr> cameras;
  cameras.emplace_back(PinholeCamera::createTestCamera());
  cameras.emplace_back(PinholeCamera::createTestCamera());
  cameras.emplace_back(PinholeCamera::createTestCamera());
  cameras.emplace_back(PinholeCamera::createTestCamera());
  NCameraId rig_id;
  generateId(&rig_id);
  // This defines an artificial camera system similar to the one on the V-Charge
  // or JanETH car.
  Position3D t_B_C0(2.0, 0.0, 0.0);
  Eigen::Matrix3d R_B_C0 = Eigen::Matrix3d::Zero();
  R_B_C0(1, 0) = -1.0;
  R_B_C0(2, 1) = -1.0;
  R_B_C0(0, 2) = 1.0;
  Quaternion q_B_C0(R_B_C0);
  Position3D t_B_C1(0.0, 1.0, 0.0);
  Eigen::Matrix3d R_B_C1 = Eigen::Matrix3d::Zero();
  R_B_C1(0, 0) = 1.0;
  R_B_C1(2, 1) = -1.0;
  R_B_C1(1, 2) = 1.0;
  Quaternion q_B_C1(R_B_C1);
  Position3D t_B_C2(-1.0, 0.0, 0.0);
  Eigen::Matrix3d R_B_C2 = Eigen::Matrix3d::Zero();
  R_B_C2(1, 0) = 1.0;
  R_B_C2(2, 1) = -1.0;
  R_B_C2(0, 2) = -1.0;
  Quaternion q_B_C2(R_B_C2);
  Position3D t_B_C3(0.0, -1.0, 0.0);
  Eigen::Matrix3d R_B_C3 = Eigen::Matrix3d::Zero();
  R_B_C3(0, 0) = -1.0;
  R_B_C3(2, 1) = -1.0;
  R_B_C3(1, 2) = -1.0;
  Quaternion q_B_C3(R_B_C3);
  TransformationVector rig_transformations;
  rig_transformations.emplace_back(q_B_C0.inverse(), -t_B_C0);
  rig_transformations.emplace_back(q_B_C1.inverse(), -t_B_C1);
  rig_transformations.emplace_back(q_B_C2.inverse(), -t_B_C2);
  rig_transformations.emplace_back(q_B_C3.inverse(), -t_B_C3);
  std::string description = "Artificial Planar 4-Pinhole-Camera-Rig";
  return std::move(aligned_unique<aslam::NCamera>(
      rig_id, rig_transformations, cameras, description));
}

}  // namespace aslam
