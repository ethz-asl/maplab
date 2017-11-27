#include "vio-common/test/vio-update-simulation.h"

#include <vector>

#include <aslam/cameras/ncamera.h>
#include <aslam/common/memory.h>
#include <aslam/common/pose-types.h>
#include <aslam/common/time.h>
#include <aslam/frames/visual-nframe.h>
#include <glog/logging.h>
#include <simulation/generic-path-generator.h>
#include <simulation/visual-nframe-simulator.h>

#include "vio-common/vio-types.h"
#include "vio-common/vio-update.h"

namespace vio {

VioUpdateSimulation::VioUpdateSimulation()
    : has_simulated_vio_updates_(false) {}

VioUpdateSimulation::VioUpdateSimulation(
    const test_trajectory_gen::PathAndLandmarkSettings& path_settings)
    : has_simulated_vio_updates_(false), path_settings_(path_settings) {}

void VioUpdateSimulation::generateVioUpdates() {
  test_trajectory_gen::GenericPathGenerator path_generator(path_settings_);
  path_generator.generatePath();
  path_generator.generateLandmarks();

  // Get needed data.
  constexpr size_t kNumberOfCameras = 1u;
  n_camera_ = aslam::NCamera::createTestNCamera(kNumberOfCameras);
  const Eigen::VectorXd& timestamps = path_generator.getTimestampsInSeconds();
  path_generator.getGroundTruthTransformations(&transformation_vector_);
  velocities_ = path_generator.getTrueVelocities();
  imu_data_ = path_generator.getImuData();
  acc_bias_ = path_generator.getTrueAccBias();
  gyro_bias_ = path_generator.getTrueGyroBias();

  // Generate VisualNFrames.
  constexpr double kKeypointSigmaX = 0.1;
  constexpr bool kAddNoiseToKeypoints = false;
  constexpr size_t kNumBitsToFlip = 0u;
  simulation::VisualNFrameSimulator n_frame_simulator;
  n_frame_simulator.simulateVisualNFrames(
      timestamps, transformation_vector_, path_generator.getLandmarks(),
      n_camera_, kKeypointSigmaX, kAddNoiseToKeypoints, kNumBitsToFlip,
      &n_frame_list_, nullptr);
  int number_of_n_frames = static_cast<int>(n_frame_list_.size());
  CHECK_EQ(number_of_n_frames, timestamps.rows());
  CHECK_EQ(number_of_n_frames, static_cast<int>(transformation_vector_.size()));
  CHECK_EQ(number_of_n_frames, velocities_.cols());
  CHECK_EQ(number_of_n_frames, imu_data_.cols());
  CHECK_EQ(number_of_n_frames, acc_bias_.cols());
  CHECK_EQ(number_of_n_frames, gyro_bias_.cols());

  number_of_landmarks_ = n_frame_simulator.getGroundTruthLandmarkIds().size();

  // Generate VioUpdates.
  vio_update_list_.clear();
  size_t index = 0u;
  for (const aslam::VisualNFrame::Ptr& n_frame : n_frame_list_) {
    // Create SynchronizedNFrameImu.
    vio::SynchronizedNFrameImu::Ptr synced_n_frame_imu =
        aligned_shared<vio::SynchronizedNFrameImu>();
    synced_n_frame_imu->nframe = n_frame;
    if (index > 0u) {
      // Add current and previous imu measurement.
      synced_n_frame_imu->imu_timestamps.resize(Eigen::NoChange, 2);
      synced_n_frame_imu->imu_timestamps(0) =
          aslam::time::secondsToNanoSeconds(timestamps[index - 1]);
      synced_n_frame_imu->imu_timestamps(1) =
          aslam::time::secondsToNanoSeconds(timestamps[index]);
      synced_n_frame_imu->imu_measurements.resize(Eigen::NoChange, 2);
      synced_n_frame_imu->imu_measurements.col(0) = imu_data_.col(index - 1);
      synced_n_frame_imu->imu_measurements.col(1) = imu_data_.col(index);
    }
    synced_n_frame_imu->motion_wrt_last_nframe =
        vio::MotionType::kGeneralMotion;

    // Create and fill VioUpdate.
    vio::VioUpdate::Ptr vio_update = aligned_shared<vio::VioUpdate>();
    vio_update->timestamp_ns =
        aslam::time::secondsToNanoSeconds(timestamps[index]);
    vio_update->vio_state = vio::EstimatorState::kRunning;
    vio_update->vio_update_type = vio::UpdateType::kNormalUpdate;
    vio_update->keyframe_and_imudata = synced_n_frame_imu;

    vio_update->vinode.setAccBias(acc_bias_.col(index));
    vio_update->vinode.setGyroBias(gyro_bias_.col(index));
    vio_update->vinode.set_T_M_I(transformation_vector_[index]);
    vio_update->vinode.set_v_M_I(velocities_.col(index));

    vio_update->localization_state = vio::LocalizationState::kUninitialized;
    // Entry vio_update->T_G_M stays at default (identity).

    vio_update_list_.push_back(vio_update);

    ++index;
  }

  VLOG(5) << "Created " << vio_update_list_.size() << " VioUpdates.";
  has_simulated_vio_updates_ = true;
}

}  // namespace vio
