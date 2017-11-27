#include <cmath>
#include <random>
#include <stdlib.h>

#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/camera.h>
#include <aslam/common/feature-descriptor-ref.h>
#include <aslam/common/memory.h>
#include <aslam/common/pose-types.h>
#include <aslam/common/time.h>
#include <aslam/common/unique-id.h>
#include <aslam/frames/visual-frame.h>
#include <aslam/frames/visual-nframe.h>
#include <maplab-common/parallel-process.h>
#include <maplab-common/unique-id.h>

#include "simulation/visual-nframe-simulator-channels.h"
#include "simulation/visual-nframe-simulator.h"

namespace simulation {

void VisualNFrameSimulator::generateGroundTruth(size_t num_landmarks) {
  // This creates a number of random ground truth descriptors.
  ground_truth_landmark_descriptors_ = aslam::VisualFrame::DescriptorsT::Random(
      kDescriptorSizeBytes, num_landmarks);

  // Samples landmark scores between 1 and 1000.
  ground_truth_landmark_scores_ = Eigen::VectorXd::Zero(num_landmarks);
  const size_t kLandmarkSeed = 10;
  std::mt19937 gen(kLandmarkSeed);
  std::uniform_int_distribution<> dis(1, 1000);
  for (size_t i = 0; i < num_landmarks; ++i) {
    ground_truth_landmark_scores_(i) = dis(gen);
  }

  // This generates the landmark Ids.
  ground_truth_landmark_ids_.reserve(num_landmarks);
  for (size_t i = 0; i < num_landmarks; ++i) {
    vi_map::LandmarkId landmark_id;
    common::generateId(&landmark_id);
    ground_truth_landmark_ids_.emplace_back(landmark_id);
  }
}

void VisualNFrameSimulator::simulateVisualNFrames(
    const Eigen::VectorXd& timestamps_seconds,
    const aslam::TransformationVector& T_G_Bs,
    const Eigen::Matrix3Xd& G_landmarks, const aslam::NCamera::Ptr& camera_rig,
    double keypoint_sigma_px, bool add_noise_to_keypoints,
    size_t num_bits_to_flip, aslam::VisualNFrame::PtrVector* nframe_list,
    aslam::TransformationVector* poses_without_keypoints) {
  CHECK(camera_rig != nullptr);
  CHECK_NOTNULL(nframe_list);
  // Pointer poses_without_keypoints is NULL if all poses contain keypoints.
  CHECK_GT(keypoint_sigma_px, 0.0)
      << "Please provide a valid uncertainty as it is used"
      << "to set the keypoint uncertainty channel.";
  CHECK_EQ(timestamps_seconds.rows(), static_cast<int>(T_G_Bs.size()));
  nframe_list->clear();

  // Create the std. dev. 2 x 2 matrix for sampling keypoint noise.
  Eigen::Matrix2d keypoint_std_dev = Eigen::Matrix2d::Identity();
  if (add_noise_to_keypoints) {
    // Computes the noise matrix for sampling afterwards.
    Eigen::Vector2d keypoint_std_dev_diagonal =
        Eigen::Vector2d::Constant(keypoint_sigma_px);
    keypoint_std_dev = keypoint_std_dev_diagonal.asDiagonal();
  }

  size_t num_timestamps = timestamps_seconds.rows();
  size_t num_samples = static_cast<size_t>(timestamps_seconds.rows());
  nframe_list->resize(num_samples);
  CHECK_EQ(num_samples, num_timestamps)
      << "Mismatch between the number of pose samples in the "
         "generated trajectory and the number of associated timestamps.";

  size_t num_landmarks = static_cast<size_t>(G_landmarks.cols());
  generateGroundTruth(num_landmarks);
  ground_truth_landmark_observation_count_.resize(num_landmarks, 0);

  CHECK_EQ(num_samples, T_G_Bs.size())
      << "Mismatch between the number of pose samples in the "
         "generated trajectory and the number of associated aslam "
         "trajectories.";

  size_t num_cameras = camera_rig->getNumCameras();

  auto createVisualNFrameForPose = [&](
      const std::vector<size_t>& sample_idx_range) {
    for (size_t sample_idx : sample_idx_range) {
      CHECK_LT(sample_idx, num_samples);

      // This creates an new visual nframe.
      aslam::NFramesId nframe_id;
      nframe_id.randomize();
      aslam::VisualNFrame::Ptr nframe =
          aligned_shared<aslam::VisualNFrame>(nframe_id, camera_rig);

      // Fill in a frame for each camera in the rig.
      for (size_t camera_idx = 0; camera_idx < num_cameras; ++camera_idx) {
        // Sets up the frame with the timestamp, id, assigns the cameras, etc.
        // TODO(mbuerki): abstract this away.
        aslam::VisualFrame::Ptr frame(new aslam::VisualFrame);
        frame->setTimestampNanoseconds(
            aslam::time::secondsToNanoSeconds(timestamps_seconds(sample_idx)));

        aslam::FrameId frame_id;
        frame_id.randomize();
        frame->setId(frame_id);

        const aslam::Camera::ConstPtr& camera =
            camera_rig->getCameraShared(camera_idx);
        CHECK(camera);

        frame->setCameraGeometry(camera);
        // Adding an extra channel for sequential Ids. This helps during
        // development and debugging
        // as it prevents from looking at 128bit random numbers.
        nframe_channels::addSequentialId(frame.get());
        nframe_channels::setSequentialId(sample_idx, frame.get());

        nframe->setFrame(camera_idx, frame);
      }

      // Project all landmarks into the frames, add noise where necessary.
      const aslam::Transformation& T_G_B = T_G_Bs[sample_idx];
      if (add_noise_to_keypoints) {
        projectLandmarksAndFillFrame(
            G_landmarks, T_G_B.inverse(), keypoint_sigma_px, num_bits_to_flip,
            &keypoint_std_dev, nframe.get(), poses_without_keypoints);
      } else {
        projectLandmarksAndFillFrame(
            G_landmarks, T_G_B.inverse(), keypoint_sigma_px, num_bits_to_flip,
            nullptr, nframe.get(), poses_without_keypoints);
      }
      (*nframe_list)[sample_idx] = nframe;
    }
  };

  // Iterate over all samples, create and fill the VisualNFrames.
  LOG(INFO) << "Simulating VisualNFrames... ";
  const bool kAlwaysParallelize = true;
  const size_t num_threads = common::getNumHardwareThreads();
  common::ParallelProcess(
      num_samples, createVisualNFrameForPose, kAlwaysParallelize, num_threads);
}

// WARNING: This method gets executed by multiple threads. Make sure all
// variable accesses are
//          synchronized where needed.
void VisualNFrameSimulator::projectLandmarksAndFillFrame(
    const Eigen::Matrix3Xd& G_landmarks, const aslam::Transformation& T_B_G,
    double keypoint_sigma_px, size_t num_bits_to_flip,
    const Eigen::Matrix2d* keypoint_std_dev, aslam::VisualNFrame* nframe,
    aslam::TransformationVector* poses_with_no_visible_landmarks) {
  // keypoint_std_dev may be null if no noise should be added to the keypoints.
  CHECK_NOTNULL(nframe);
  // poses_with_no_visible_landmarks may be null if they are not requested.

  const bool fill_in_poses_without_keypoints =
      poses_with_no_visible_landmarks != nullptr;
  const bool add_noise_to_keypoints = keypoint_std_dev != nullptr;

  // Keypoint noise generator.
  const double kMean = 0.0;
  const size_t kRandomSeed = 0.0;
  std::default_random_engine random_generator(kRandomSeed);
  std::normal_distribution<double> keypoint_noise_generator(
      kMean, keypoint_sigma_px);

  const size_t num_landmarks = static_cast<size_t>(G_landmarks.cols());
  const size_t num_cameras = nframe->getNumCameras();
  for (size_t camera_idx = 0; camera_idx < num_cameras; ++camera_idx) {
    const aslam::Camera& camera = nframe->getCamera(camera_idx);

    // Project all landmarks into the camera.
    aslam::Transformation T_C_G = nframe->get_T_C_B(camera_idx) * T_B_G;
    Eigen::Matrix3Xd C_landmarks = T_C_G.transformVectorized(G_landmarks);

    Eigen::Matrix2Xd all_keypoints;
    std::vector<aslam::ProjectionResult> projection_results;
    camera.project3Vectorized(C_landmarks, &all_keypoints, &projection_results);
    CHECK_EQ(num_landmarks, static_cast<size_t>(C_landmarks.cols()));

    // Add keypoint noise.
    if (add_noise_to_keypoints) {
      all_keypoints = all_keypoints.unaryExpr([&](double x) {
        return x + keypoint_noise_generator(random_generator);
      });
    }

    // Initializes matrices for the case where all landmarks would be visible.
    Eigen::Matrix2Xd frame_keypoints;
    frame_keypoints.resize(Eigen::NoChange, num_landmarks);
    aslam::VisualFrame::DescriptorsT frame_descriptors;
    frame_descriptors.resize(kDescriptorSizeBytes, num_landmarks);
    nframe_channels::GroundTruthLandmarkIds frame_ground_truth_landmark_ids;
    frame_ground_truth_landmark_ids.resize(num_landmarks);
    Eigen::VectorXd frame_keypoint_scores;
    frame_keypoint_scores.resize(num_landmarks);
    Eigen::VectorXi frame_track_ids;
    frame_track_ids.resize(num_landmarks);

    // Check all keypoints for visibility and add it to the frame.
    size_t visible_idx = 0;
    for (size_t landmark_idx = 0; landmark_idx < num_landmarks;
         ++landmark_idx) {
      bool keypoint_visible =
          withinImageBoxWithBorder(
              all_keypoints.block<2, 1>(0, landmark_idx), camera) &&
          (projection_results[landmark_idx].getDetailedStatus() !=
           aslam::ProjectionResult::POINT_BEHIND_CAMERA) &&
          (projection_results[landmark_idx].getDetailedStatus() !=
           aslam::ProjectionResult::PROJECTION_INVALID);

      if (keypoint_visible) {
        // Add keypoint, score and landmark id.
        frame_keypoints.col(visible_idx) = all_keypoints.col(landmark_idx);
        frame_keypoint_scores(visible_idx) =
            ground_truth_landmark_scores_(landmark_idx);
        frame_ground_truth_landmark_ids(visible_idx) = landmark_idx;
        frame_track_ids(visible_idx) = landmark_idx;

        // Add descriptor and add noise if requested.
        frame_descriptors.col(visible_idx) =
            ground_truth_landmark_descriptors_.col(landmark_idx);

        if (num_bits_to_flip > 0) {
          const bool kMemoryOwned = false;
          aslam::common::FeatureDescriptorRef descriptor_wrap(
              &(frame_descriptors.coeffRef(0, visible_idx)),
              kDescriptorSizeBytes, kMemoryOwned);
          aslam::common::FlipNRandomBits(num_bits_to_flip, &descriptor_wrap);
        }
        {
          std::unique_lock<std::mutex> lock(m_parallel_projection_);
          ++ground_truth_landmark_observation_count_[landmark_idx];
        }
        ++visible_idx;
      }
    }

    const size_t num_visible_landmarks = visible_idx;
    aslam::VisualFrame::Ptr frame = nframe->getFrameShared(camera_idx);
    CHECK(frame);

    // Resize matrices to the number of visible landmarks before assigning to
    // the channels.
    frame_keypoints.conservativeResize(Eigen::NoChange, num_visible_landmarks);
    frame->swapKeypointMeasurements(&frame_keypoints);

    frame_descriptors.conservativeResize(
        Eigen::NoChange, num_visible_landmarks);
    frame->swapDescriptors(&frame_descriptors);

    frame_keypoint_scores.conservativeResize(
        num_visible_landmarks, Eigen::NoChange);
    frame->swapKeypointScores(&frame_keypoint_scores);

    frame_track_ids.conservativeResize(num_visible_landmarks, Eigen::NoChange);
    frame->swapTrackIds(&frame_track_ids);

    frame_ground_truth_landmark_ids.conservativeResize(
        num_visible_landmarks, Eigen::NoChange);
    nframe_channels::addGroundTruthLandmarkIds(frame.get());
    nframe_channels::swapGroundTruthLandmarkIds(
        &frame_ground_truth_landmark_ids, frame.get());

    // The keypoint uncertainty is set to a fix value, regardless of whether
    // there is actually
    // any noise on the keypoints or not.
    Eigen::VectorXd keypoint_uncertainties = Eigen::VectorXd::Constant(
        num_visible_landmarks, keypoint_sigma_px * keypoint_sigma_px);
    frame->swapKeypointMeasurementUncertainties(&keypoint_uncertainties);

    // Fill in the blind poses.
    if (num_visible_landmarks == 0u) {
      LOG(WARNING) << "Zero visible landmarks for frame with sequential id "
                   << nframe_channels::getSequentialId(*frame) << " (camera "
                   << camera_idx << ").";
      if (fill_in_poses_without_keypoints) {
        aslam::Transformation T_C_G = nframe->get_T_C_B(camera_idx) * T_B_G;
        {
          std::unique_lock<std::mutex> lock(m_parallel_projection_);
          poses_with_no_visible_landmarks->push_back(T_C_G.inverse());
        }
      }
    }
  }
}

const aslam::VisualFrame::DescriptorsT&
VisualNFrameSimulator::getGroundTruthDescriptors() const {
  CHECK_GT(ground_truth_landmark_descriptors_.cols(), 0)
      << "Seems like no ground truth "
         "descriptors have been generated yet.";
  return ground_truth_landmark_descriptors_;
}

const vi_map::LandmarkIdList&
VisualNFrameSimulator::getGroundTruthLandmarkIds() const {
  CHECK_GT(ground_truth_landmark_ids_.size(), 0u)
      << "Seems like no ground truth "
         "landmark ids have been generated yet.";
  return ground_truth_landmark_ids_;
}

const Eigen::VectorXd& VisualNFrameSimulator::getGroundTruthLandmarkScores()
    const {
  CHECK_GT(ground_truth_landmark_scores_.rows(), 0)
      << "Seems like no ground truth "
         "landmark scores have been generated yet.";
  return ground_truth_landmark_scores_;
}

}  // namespace simulation
