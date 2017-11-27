#ifndef VIO_COMMON_VIO_UPDATE_SIMULATION_H_
#define VIO_COMMON_VIO_UPDATE_SIMULATION_H_

#include <vector>

#include <aslam/cameras/ncamera.h>
#include <aslam/frames/visual-nframe.h>
#include <simulation/generic-path-generator.h>

#include "vio-common/vio-update.h"

namespace vio {

// Creates simulated VioUpdates to be used for testing.
class VioUpdateSimulation {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::vector<vio::VioUpdate::Ptr> VioUpdateVector;

  VioUpdateSimulation();
  explicit VioUpdateSimulation(
      const test_trajectory_gen::PathAndLandmarkSettings& path_settings);

  void generateVioUpdates();

  bool hasSimulatedVioUpdates() const {
    return has_simulated_vio_updates_;
  }

  size_t getNumberOfVioUpdates() const {
    CHECK(has_simulated_vio_updates_);
    return vio_update_list_.size();
  }

  vio::VioUpdate::Ptr getVioUpdate(const size_t index) const {
    CHECK_LT(index, vio_update_list_.size());
    CHECK(has_simulated_vio_updates_);
    return vio_update_list_[index];
  }

  const VioUpdateVector& getAllVioUpdates() const {
    CHECK(has_simulated_vio_updates_);
    return vio_update_list_;
  }

  const Eigen::Matrix3Xd& getAccBias() const {
    CHECK(has_simulated_vio_updates_);
    return acc_bias_;
  }

  const Eigen::Matrix3Xd& getGyroBias() const {
    CHECK(has_simulated_vio_updates_);
    return gyro_bias_;
  }

  const Eigen::Matrix<double, 6, Eigen::Dynamic>& getImuData() const {
    CHECK(has_simulated_vio_updates_);
    return imu_data_;
  }

  const aslam::NCamera::Ptr& getNCamera() const {
    CHECK(has_simulated_vio_updates_);
    return n_camera_;
  }

  const aslam::VisualNFrame::PtrVector& getNFrameList() const {
    CHECK(has_simulated_vio_updates_);
    return n_frame_list_;
  }

  const aslam::TransformationVector& getTransformationVector() const {
    CHECK(has_simulated_vio_updates_);
    return transformation_vector_;
  }

  const Eigen::Matrix3Xd& getVelocities() const {
    CHECK(has_simulated_vio_updates_);
    return velocities_;
  }

  size_t getNumberOfLandmarks() const {
    CHECK(has_simulated_vio_updates_);
    return number_of_landmarks_;
  }

 private:
  bool has_simulated_vio_updates_;

  // Contains all simulated VioUpdates.
  VioUpdateVector vio_update_list_;

  Eigen::Matrix3Xd acc_bias_;
  Eigen::Matrix3Xd gyro_bias_;
  Eigen::Matrix<double, 6, Eigen::Dynamic> imu_data_;
  aslam::NCamera::Ptr n_camera_;
  aslam::VisualNFrame::PtrVector n_frame_list_;
  aslam::TransformationVector transformation_vector_;
  Eigen::Matrix3Xd velocities_;
  size_t number_of_landmarks_;

  test_trajectory_gen::PathAndLandmarkSettings path_settings_;
};

}  // namespace vio

#endif  // VIO_COMMON_VIO_UPDATE_SIMULATION_H_
