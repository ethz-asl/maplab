#include <cstdlib>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/camera.h>
#include <aslam/cameras/distortion-fisheye.h>
#include <aslam/cameras/distortion.h>
#include <aslam/cameras/ncamera.h>
#include <aslam/matcher/match.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <maplab-common/pose_types.h>
#include <maplab-common/quaternion-math.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <simulation/generic-path-generator.h>
#include <simulation/visual-nframe-simulator-channels.h>
#include <simulation/visual-nframe-simulator.h>

#include "geometric-vision/rotation-only-detector.h"

class RotationOnlyDetectorTest
    : public ::testing::TestWithParam<std::vector<double> > {
 protected:
  virtual void SetUp() {}
  test_trajectory_gen::PathAndLandmarkSettings settings_;
  const double kMotionClassificationThreshold = 1.0e-3;
  std::unique_ptr<geometric_vision::RotationOnlyDetector> detector_;
};

// Parameters:
// [1] Path (3 = translation-only, 4 = rotation-only).
// [2] Number of cameras.
// [3] Noise level [pixel].
INSTANTIATE_TEST_CASE_P(
    Parameters, RotationOnlyDetectorTest,
    ::testing::Values(
        std::vector<double>{3.0, 1.0, 0.0}, std::vector<double>{4.0, 1.0, 0.0}, // NOLINT
        std::vector<double>{3.0, 2.0, 0.0}, std::vector<double>{4.0, 2.0, 0.0}, // NOLINT
        std::vector<double>{3.0, 1.0, 0.8}, std::vector<double>{4.0, 1.0, 0.8}, // NOLINT
        std::vector<double>{3.0, 2.0, 0.8}, std::vector<double>{4.0, 2.0, 0.8})); // NOLINT

// *************************************************************
// The test goes as follows:
// [1] Generate the cameras.
// [2] Generate the path and landmarks.
// [3] Setup and run the visual nframe simulator.
// [4] Grab the ground truth matches.
// [5] Run and validate the rotation-only detector.
// *************************************************************
TEST_P(RotationOnlyDetectorTest, testRotationOnlyDetector) {
  // Extract the current parameters.
  std::vector<double> param = GetParam();
  test_trajectory_gen::Path param_path_type =
      static_cast<test_trajectory_gen::Path>(param[0]);
  size_t param_num_cameras = static_cast<size_t>(param[1]);
  double param_noise_level = param[2];

  // **************************************************
  // [1] Generate the cameras.
  // **************************************************
  // Set the camera intrinsics.
  const double fu = 200.0;
  const double fv = 200.0;
  const unsigned int ru = 640;
  const unsigned int rv = 480;
  const double cu = ru / 2.0;
  const double cv = rv / 2.0;

  Eigen::VectorXd intrinsics(4);
  intrinsics << fu, fv, cu, cv;

  // Generate the cameras: Test cameras but without distortion.
  aslam::TransformationVector T_C_B_vector;
  std::vector<aslam::Camera::Ptr> camera_vector;
  for (size_t camera_idx = 0; camera_idx < param_num_cameras; ++camera_idx) {
    aslam::Camera::Ptr camera = std::shared_ptr<aslam::PinholeCamera>(
        new aslam::PinholeCamera(intrinsics, ru, rv));
    CHECK(camera);
    aslam::CameraId cam_id;
    cam_id.randomize();
    camera->setId(cam_id);
    camera_vector.push_back(camera);

    // Offset each camera 0.1 m in x direction and rotate it to face forward.
    Eigen::Vector3d position(0.1 * camera_idx, 0.0, 0.0);
    aslam::Quaternion q_C_B(0.5, 0.5, -0.5, 0.5);
    aslam::Transformation T_C_B(q_C_B, position);
    T_C_B_vector.push_back(T_C_B);
  }

  // Construct the ID object.
  aslam::NCameraId n_camera_id;

  // Randomize the ID.
  n_camera_id.randomize();

  // Generate the ncamera object.
  aslam::NCamera::Ptr n_camera;
  n_camera.reset(
      new aslam::NCamera(
          n_camera_id, T_C_B_vector, camera_vector, "pose_estimator_test"));

  // **************************************************
  // [2] Generate the path and landmarks.
  // **************************************************
  // Generate the path.
  settings_.mode = param_path_type;
  settings_.gravity_meter_by_second2 = 9.81;
  settings_.landmark_seed = 5u;
  settings_.landmark_variance_meter = 0.25;
  settings_.num_of_landmarks = 5000;
  settings_.sampling_time_second = 0.1;

  std::shared_ptr<test_trajectory_gen::GenericPathGenerator> path_generator;
  path_generator =
      aligned_shared<test_trajectory_gen::GenericPathGenerator>(settings_);
  path_generator->generatePath();
  path_generator->generateLandmarks();

  // Get ground truth transformations between frames.
  aslam::TransformationVector true_T_G_B;
  path_generator->getGroundTruthTransformations(&true_T_G_B);

  // **************************************************
  // [3] Setup and run visual nframe simulator.
  // **************************************************
  simulation::VisualNFrameSimulator nframe_simulator;
  aslam::VisualNFrame::PtrVector nframes;
  bool add_noise_to_keypoints = false;
  double keypoint_sigma_pixel = 0.8;
  if (param_noise_level > 0.0) {
    add_noise_to_keypoints = true;
    keypoint_sigma_pixel = param_noise_level;
  }
  const size_t kNumBitsToFlip = 0u;

  aslam::TransformationVector T_G_Bs;
  path_generator->getGroundTruthTransformations(&T_G_Bs);
  aslam::TransformationVector poses_without_keypoints;
  nframe_simulator.simulateVisualNFrames(
      path_generator->getTimestampsInSeconds(), T_G_Bs,
      path_generator->getLandmarks(), n_camera, keypoint_sigma_pixel,
      add_noise_to_keypoints, kNumBitsToFlip, &nframes,
      &poses_without_keypoints);
  ASSERT_GT(nframes.size(), 0) << "Zero simulated nframes. Aborting.";

  // ********************************************************************
  // [4] Get ground truth track IDs and extract the ground truth matches.
  // ********************************************************************
  for (size_t time_idx = 0; time_idx < nframes.size(); ++time_idx) {
    const aslam::VisualNFrame::Ptr& nframe = nframes[time_idx];
    for (size_t camera_idx = 0; camera_idx < nframe->getNumCameras();
         ++camera_idx) {
      aslam::VisualFrame::Ptr frame = nframe->getFrameShared(camera_idx);
      CHECK(frame);
      const Eigen::VectorXi& groundtruth_ids =
          simulation::nframe_channels::getGroundTruthLandmarkIds(*frame);
      frame->setTrackIds(groundtruth_ids);
      CHECK_EQ(
          frame->getTrackIds().rows(), frame->getNumKeypointMeasurements());
    }
  }

  detector_.reset(
      new geometric_vision::RotationOnlyDetector(
          kMotionClassificationThreshold));
  // Loop over frames in trajectory: The acceleration is limited by the path
  // planner and thus
  // the first few frames are a mixture of translation and rotation. Either
  // increase the maximal
  // acceleration / velocity constraint or skip the first frames (as done here).
  const size_t kStartFrameIdx = 3u;
  for (size_t frame_idx = kStartFrameIdx; frame_idx < nframes.size() - 1;
       ++frame_idx) {
    size_t frame_idx_k = frame_idx;
    size_t frame_idx_kp1 = frame_idx + 1;

    aslam::FrameToFrameMatchesList matches_kp1_k;
    for (size_t camera_idx = 0; camera_idx < param_num_cameras; ++camera_idx) {
      aslam::FrameToFrameMatches matches;
      aslam::VisualFrame::Ptr frame_k =
          nframes[frame_idx_k]->getFrameShared(camera_idx);
      aslam::VisualFrame::Ptr frame_kp1 =
          nframes[frame_idx_kp1]->getFrameShared(camera_idx);
      const Eigen::VectorXi& track_ids_k = frame_k->getTrackIds();
      const Eigen::VectorXi& track_ids_kp1 = frame_kp1->getTrackIds();
      typedef std::unordered_map<int, size_t> TrackIdKeypointIdxMap;
      TrackIdKeypointIdxMap track_id_keypoint_idx_map_kp1;
      for (int keypoint_idx_kp1 = 0; keypoint_idx_kp1 < track_ids_kp1.rows();
           ++keypoint_idx_kp1) {
        int track_id_kp1 = track_ids_kp1(keypoint_idx_kp1);
        CHECK_NE(track_id_kp1, -1)
            << "There should be no unassociated keypoints for simulated data.";
        track_id_keypoint_idx_map_kp1.insert(
            std::make_pair(track_id_kp1, keypoint_idx_kp1));
      }
      for (int keypoint_idx_k = 0; keypoint_idx_k < track_ids_k.rows();
           ++keypoint_idx_k) {
        int track_id_k = track_ids_k(keypoint_idx_k);
        TrackIdKeypointIdxMap::const_iterator it =
            track_id_keypoint_idx_map_kp1.find(track_id_k);
        if (it != track_id_keypoint_idx_map_kp1.end()) {
          size_t keypoint_idx_kp1 = it->second;
          CHECK_NE(keypoint_idx_k, -1);
          matches.emplace_back(keypoint_idx_kp1, keypoint_idx_k);
        }
      }
      LOG(WARNING) << "Extracted " << matches.size()
                   << " matches from externally set "
                   << "track id channel.";
      matches_kp1_k.emplace_back(matches);
    }

    // **************************************************
    // [5] Run and validate the rotation-only detector.
    // **************************************************
    aslam::Transformation true_T_Bkp1_Bk =
        true_T_G_B[frame_idx_kp1].inverse() * true_T_G_B[frame_idx_k];
    bool remove_outliers = false;
    if (add_noise_to_keypoints) {
      remove_outliers = true;
    }
    const bool is_only_rotated = detector_->isRotationOnlyMotion(
        nframes[frame_idx_kp1], nframes[frame_idx_k], matches_kp1_k,
        true_T_Bkp1_Bk.getRotation(), remove_outliers);
    if (param_path_type == test_trajectory_gen::Path::kRotationOnly &&
        frame_idx_kp1 > kStartFrameIdx + 1) {
      LOG(INFO) << "Frame: " << frame_idx_kp1 << "/" << nframes.size()
                << ", Rotation-only motion, threshold = "
                << kMotionClassificationThreshold
                << ", Deviation from parallel bearing vectors = "
                << detector_->getDeviationFromParallelBearingVectorLastUpdate()
                << ", rotation-only motion detected? " << is_only_rotated;
      EXPECT_TRUE(is_only_rotated);
    } else if (
        param_path_type == test_trajectory_gen::Path::kTranslationOnly &&
        frame_idx_kp1 > kStartFrameIdx + 1) {
      LOG(INFO) << "Frame: " << frame_idx_kp1 << "/" << nframes.size()
                << ", Translation-only motion, threshold = "
                << kMotionClassificationThreshold
                << ", Deviation from parallel bearing vectors = "
                << detector_->getDeviationFromParallelBearingVectorLastUpdate()
                << ", rotation-only motion detected? " << is_only_rotated;
      EXPECT_FALSE(is_only_rotated);
    }
  }
}
MAPLAB_UNITTEST_ENTRYPOINT
