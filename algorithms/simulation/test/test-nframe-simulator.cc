#include <iomanip>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <aslam/common/feature-descriptor-ref.h>
#include <aslam/common/memory.h>
#include <aslam/common/statistics/statistics.h>
#include <aslam/frames/visual-nframe.h>
#include <eigen-checks/gtest.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <maplab-common/feature-descriptor-ref.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

#include "simulation/generic-path-generator.h"
#include "simulation/visual-nframe-simulator-channels.h"
#include "simulation/visual-nframe-simulator.h"

namespace simulation {

class VisualNFrameSimulatorTest
    : public ::testing::TestWithParam<test_trajectory_gen::Path> {
 protected:
  virtual void SetUp() {
    num_cameras_ = 1;
    settings_.imu_sigmas.acc_bias_random_walk_noise_density = 0.1;
    settings_.imu_sigmas.acc_noise_density = 0.1;
    settings_.gravity_meter_by_second2 = 9.81;
    settings_.imu_sigmas.gyro_bias_random_walk_noise_density = 0.1;
    settings_.imu_sigmas.gyro_noise_density = 0.1;
    settings_.imu_noise_bias_seed = 10u;
    settings_.num_of_path_constraints = 8;
    settings_.landmark_seed = 5u;
    settings_.landmark_variance_meter = 0.25;
    settings_.num_of_landmarks = 1500;
    settings_.sampling_time_second = 0.1;
    settings_.distance_to_keypoints_meter = 7.5;

    for (size_t i = 0; i < 10; ++i) {
      settings_.pose_waypoints.emplace_back(i / 5.0, i / 5.0, 0.0, 0.0);
    }

    // Create the camera rig.
    camera_rig_ = aslam::NCamera::createTestNCamera(num_cameras_);
  }
  test_trajectory_gen::PathAndLandmarkSettings settings_;
  std::shared_ptr<test_trajectory_gen::GenericPathGenerator> path_generator_;
  aslam::NCamera::Ptr camera_rig_;
  simulation::VisualNFrameSimulator nframe_simulator_;
  aslam::TransformationVector ground_truth_transformations_;

  Eigen::Matrix3Xd G_landmarks_;

  size_t num_cameras_;
  size_t num_landmarks_;
};

// Selects the desired trajectory
// - path from circle equation:  mode = 1
// - path from elliptical equation: mode = 2
// - rotation-only path: mode = 3
// - translation-only path: mode = 4
// - path from waypoints (file): mode = 5
// - path from waypoints (ctor): mode = 6.
INSTANTIATE_TEST_CASE_P(
    PathModes, VisualNFrameSimulatorTest,
    ::testing::Values(
        test_trajectory_gen::Path::kCircular,
        test_trajectory_gen::Path::kElliptical,
        test_trajectory_gen::Path::kRotationOnly,
        test_trajectory_gen::Path::kTranslationOnly,
        test_trajectory_gen::Path::kFromFile,
        test_trajectory_gen::Path::kFromCtor));

TEST_P(VisualNFrameSimulatorTest, testSimulateNFrames_noNoise) {
  settings_.mode = GetParam();
  path_generator_ =
      aligned_shared<test_trajectory_gen::GenericPathGenerator>(settings_);

  path_generator_->generatePath();
  path_generator_->generateLandmarks();

  G_landmarks_ = path_generator_->getLandmarks();
  num_landmarks_ = static_cast<size_t>(G_landmarks_.cols());
  path_generator_->getGroundTruthTransformations(
      &ground_truth_transformations_);

  aslam::VisualNFrame::PtrVector nframes;
  bool add_noise_to_keypoints = false;
  size_t num_bits_to_flip = 0;
  const double kKeypointSigma = 0.8;

  aslam::TransformationVector T_G_Bs;
  path_generator_->getGroundTruthTransformations(&T_G_Bs);
  aslam::TransformationVector poses_without_keypoints;
  nframe_simulator_.simulateVisualNFrames(
      path_generator_->getTimestampsInSeconds(), T_G_Bs,
      path_generator_->getLandmarks(), camera_rig_, kKeypointSigma,
      add_noise_to_keypoints, num_bits_to_flip, &nframes,
      &poses_without_keypoints);
  EXPECT_TRUE(poses_without_keypoints.empty());

  const aslam::VisualFrame::DescriptorsT& ground_truth_descriptors =
      nframe_simulator_.getGroundTruthDescriptors();
  ASSERT_EQ(ground_truth_descriptors.cols(), static_cast<int>(num_landmarks_));
  const size_t num_frames = nframes.size();
  ASSERT_EQ(ground_truth_transformations_.size(), nframes.size());
  const Eigen::VectorXd& ground_truth_landmark_scores =
      nframe_simulator_.getGroundTruthLandmarkScores();

  for (size_t nframe_idx = 0; nframe_idx < num_frames; ++nframe_idx) {
    const aslam::VisualNFrame::Ptr& nframe = nframes[nframe_idx];
    const aslam::Transformation& T_G_B =
        ground_truth_transformations_[nframe_idx];
    ASSERT_EQ(camera_rig_->getNumCameras(), num_cameras_);

    for (size_t camera_idx = 0; camera_idx < num_cameras_; ++camera_idx) {
      const aslam::VisualFrame& frame = nframe->getFrame(camera_idx);
      const aslam::Camera& camera = nframe->getCamera(camera_idx);
      const aslam::Transformation& T_C_B = nframe->get_T_C_B(camera_idx);

      const Eigen::VectorXi& landmark_id_channel =
          nframe_channels::getGroundTruthLandmarkIds(frame);

      size_t num_keypoints = frame.getNumKeypointMeasurements();
      for (size_t keypoint_idx = 0; keypoint_idx < num_keypoints;
           ++keypoint_idx) {
        // Get the id of the associated landmark.s
        int keypoint_landmarkid = landmark_id_channel(keypoint_idx);
        EXPECT_LT(keypoint_landmarkid, G_landmarks_.cols());

        // Reproject the groundtruth landmark and compare to stored keypoint.
        Eigen::Vector2d keypoint_groundtruth;
        Eigen::Vector3d G_landmark = G_landmarks_.col(keypoint_landmarkid);
        Eigen::Vector3d C_landmark = T_C_B * T_G_B.inverse() * G_landmark;
        aslam::ProjectionResult res =
            camera.project3(C_landmark, &keypoint_groundtruth);

        EXPECT_EQ(
            res.getDetailedStatus(), aslam::ProjectionResult::KEYPOINT_VISIBLE);
        EXPECT_NEAR_EIGEN(
            frame.getKeypointMeasurement(keypoint_idx), keypoint_groundtruth,
            1e-12);

        double score_groundtruth =
            ground_truth_landmark_scores(keypoint_landmarkid);
        EXPECT_EQ(frame.getKeypointScore(keypoint_idx), score_groundtruth);
        EXPECT_EQ(
            frame.getKeypointMeasurementUncertainty(keypoint_idx),
            kKeypointSigma * kKeypointSigma);

        // Test descriptor.
        aslam::common::FeatureDescriptorConstRef noisy_descriptor(
            frame.getDescriptor(keypoint_idx),
            VisualNFrameSimulator::kDescriptorSizeBytes);
        aslam::common::FeatureDescriptorConstRef ground_truth_descriptor(
            &ground_truth_descriptors.coeffRef(0, keypoint_landmarkid),
            VisualNFrameSimulator::kDescriptorSizeBytes);

        EXPECT_EQ(
            aslam::common::GetNumBitsDifferent(
                noisy_descriptor, ground_truth_descriptor),
            0u);
      }
      EXPECT_GT(frame.getNumKeypointMeasurements(), 50u)
          << "Less than 50 landmarks seen in the "
             "frame. This indicates that something might be wrong (camera "
             "extrinsics, rotation "
             "conventions, etc.). Note: this is soft lower bound. The actual "
             "number of visible "
             "landmarks per frame should be around 80, fluctuating with noise.";
    }
  }
}

TEST_P(VisualNFrameSimulatorTest, testSimulateNFrames_withNoise) {
  settings_.mode = GetParam();
  path_generator_ =
      aligned_shared<test_trajectory_gen::GenericPathGenerator>(settings_);

  path_generator_->generatePath();
  path_generator_->generateLandmarks();

  G_landmarks_ = path_generator_->getLandmarks();
  num_landmarks_ = static_cast<size_t>(G_landmarks_.cols());
  path_generator_->getGroundTruthTransformations(
      &ground_truth_transformations_);

  aslam::VisualNFrame::PtrVector nframes;
  bool add_noise_to_keypoints = true;
  size_t num_bits_to_flip = 5;
  const double kKeypointSigma = 0.8;

  aslam::TransformationVector T_G_Bs;
  path_generator_->getGroundTruthTransformations(&T_G_Bs);
  aslam::TransformationVector poses_without_keypoints;
  nframe_simulator_.simulateVisualNFrames(
      path_generator_->getTimestampsInSeconds(), T_G_Bs,
      path_generator_->getLandmarks(), camera_rig_, kKeypointSigma,
      add_noise_to_keypoints, num_bits_to_flip, &nframes,
      &poses_without_keypoints);
  EXPECT_TRUE(poses_without_keypoints.empty());

  const aslam::VisualFrame::DescriptorsT& ground_truth_descriptors =
      nframe_simulator_.getGroundTruthDescriptors();
  ASSERT_EQ(ground_truth_descriptors.cols(), static_cast<int>(num_landmarks_));
  const size_t num_frames = nframes.size();
  ASSERT_EQ(ground_truth_transformations_.size(), nframes.size());

  statistics::StatsCollectorImpl stat_keypoint_noise_u("keypoint_noise_u");
  statistics::StatsCollectorImpl stat_keypoint_noise_v("keypoint_noise_v");
  for (size_t nframe_idx = 0; nframe_idx < num_frames; ++nframe_idx) {
    const aslam::VisualNFrame::Ptr& nframe = nframes[nframe_idx];
    const aslam::Transformation& T_G_B =
        ground_truth_transformations_[nframe_idx];
    ASSERT_EQ(camera_rig_->getNumCameras(), num_cameras_);

    for (size_t camera_idx = 0; camera_idx < num_cameras_; ++camera_idx) {
      const aslam::VisualFrame& frame = nframe->getFrame(camera_idx);
      const aslam::Camera& camera = nframe->getCamera(camera_idx);
      const aslam::Transformation& T_C_B = nframe->get_T_C_B(camera_idx);

      const Eigen::VectorXi& landmark_id_channel =
          nframe_channels::getGroundTruthLandmarkIds(frame);

      size_t num_keypoints = frame.getNumKeypointMeasurements();
      for (size_t keypoint_idx = 0; keypoint_idx < num_keypoints;
           ++keypoint_idx) {
        // Get the id of the associated landmark.s
        int keypoint_landmarkid = landmark_id_channel(keypoint_idx);
        EXPECT_LT(keypoint_landmarkid, G_landmarks_.cols());

        // Reproject the groundtruth landmark and compare to stored keypoint.
        Eigen::Vector2d keypoint_groundtruth;
        Eigen::Vector3d G_landmark = G_landmarks_.col(keypoint_landmarkid);
        Eigen::Vector3d C_landmark = T_C_B * T_G_B.inverse() * G_landmark;
        aslam::ProjectionResult res =
            camera.project3(C_landmark, &keypoint_groundtruth);
        EXPECT_TRUE(
            res.getDetailedStatus() !=
                aslam::ProjectionResult::PROJECTION_INVALID ||
            res.getDetailedStatus() !=
                aslam::ProjectionResult::POINT_BEHIND_CAMERA);
        Eigen::Vector2d keypoint_noise =
            keypoint_groundtruth - frame.getKeypointMeasurement(keypoint_idx);

        stat_keypoint_noise_u.AddSample(keypoint_noise(0));
        stat_keypoint_noise_v.AddSample(keypoint_noise(1));

        // Test descriptor.
        aslam::common::FeatureDescriptorConstRef noisy_descriptor(
            frame.getDescriptor(keypoint_idx),
            VisualNFrameSimulator::kDescriptorSizeBytes);
        aslam::common::FeatureDescriptorConstRef ground_truth_descriptor(
            &ground_truth_descriptors.coeffRef(0, keypoint_landmarkid),
            VisualNFrameSimulator::kDescriptorSizeBytes);

        EXPECT_EQ(
            aslam::common::GetNumBitsDifferent(
                noisy_descriptor, ground_truth_descriptor),
            num_bits_to_flip);
      }
    }
  }

  double keypoint_var = kKeypointSigma * kKeypointSigma;
  EXPECT_NEAR(
      statistics::Statistics::GetVariance("keypoint_noise_u"), keypoint_var,
      1e-1);
  EXPECT_NEAR(
      statistics::Statistics::GetVariance("keypoint_noise_v"), keypoint_var,
      3e-1);
}

}  // namespace simulation

MAPLAB_UNITTEST_ENTRYPOINT
