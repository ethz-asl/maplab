#include "geometric-vision/relative-non-central-pnp.h"

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
#include <aslam/matcher/matching-engine-exclusive.h>
#include <aslam/matcher/matching-problem-frame-to-frame.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <maplab-common/pose_types.h>
#include <maplab-common/quaternion-math.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <opengv/test/experiment_helpers.hpp>
#include <opengv/test/random_generators.hpp>
#include <opengv/test/time_measurement.hpp>
#include <simulation/generic-path-generator.h>
#include <simulation/visual-nframe-simulator-channels.h>
#include <simulation/visual-nframe-simulator.h>

class RelativeNonCentralPnpTest : public ::testing::Test {
 public:
  typedef aslam::FisheyeDistortion DistortionType;
  typedef aslam::PinholeCamera CameraType;
};

// Compares relative non-central PnP estimate from nframe interface
// with ground truth and with openGV PnP result.
TEST_F(RelativeNonCentralPnpTest, testInterfaceWithOpengvSetup) {
  // Force NOT to use a random seed.
  srand(1);

  // Set experiment parameters.
  // Solve problem with only one camera?
  constexpr bool kAsCentral = false;
  constexpr bool kRandomSeed = false;
  constexpr double kNoise = 0.0;
  constexpr double kOutlierFraction = 0.1;
  constexpr size_t kNumCams = 4;
  constexpr size_t kNumNFrames = 2;
  // Maximal number of correspondences between nframe 0 and nframe 1. In
  // general, the number of correspondences is (a lot) lower since not all
  // keypoints are seen from both nframes. Makes sure that the unit test
  // does not fail due to insufficient observation pairs.
  constexpr size_t kPointsPerCam = 5000;
  constexpr size_t kViewpoint0 = 0;
  constexpr size_t kViewpoint1 = 1;

  // Generate a "random"/arbitrary pose for viewpoint 0.
  opengv::translation_t position_nframe_0 = Eigen::Vector3d::Zero();
  opengv::rotation_t rotation_nframe_0 = Eigen::Matrix3d::Identity();

  // Generate a "random"/arbitrary pose for viewpoint 1.
  opengv::translation_t position_nframe_1 =
      opengv::generateRandomTranslation(2.0);
  opengv::rotation_t rotation_nframe_1 = opengv::generateRandomRotation(0.5);

  // Create a "random"/arbitrary camera system.
  opengv::translations_t camera_offsets;
  opengv::rotations_t camera_rotations;
  opengv::generateRandomCameraSystem(
      kNumCams, camera_offsets, camera_rotations);

  // Derive correspondences based on random point-cloud.
  std::vector<std::shared_ptr<opengv::bearingVectors_t> >
      multi_bearing_vectors_nframe_0;
  std::vector<std::shared_ptr<opengv::bearingVectors_t> >
      multi_bearing_vectors_nframe_1;
  std::vector<std::shared_ptr<Eigen::MatrixXd> > keypoints_ground_truth;

  opengv::generateMulti2D2DCorrespondences(
      position_nframe_0, rotation_nframe_0, position_nframe_1,
      rotation_nframe_1, camera_offsets, camera_rotations, kPointsPerCam,
      kNoise, kOutlierFraction, multi_bearing_vectors_nframe_0,
      multi_bearing_vectors_nframe_1, keypoints_ground_truth);

  // Extract the relative pose
  opengv::translation_t relative_translation_ground_truth;
  opengv::rotation_t relative_rotation_ground_truth;
  opengv::extractRelativePose(
      position_nframe_0, position_nframe_1, rotation_nframe_0,
      rotation_nframe_1, relative_translation_ground_truth,
      relative_rotation_ground_truth, false);

  // Define the camera parameters.
  constexpr double distortion_param = 0.0;
  constexpr double fu = 200.0;
  constexpr double fv = 200.0;
  constexpr size_t ru = 640;
  constexpr size_t rv = 480;
  constexpr double cu = ru / 2.0;
  constexpr double cv = rv / 2.0;

  // Set the camera parameters.
  Eigen::VectorXd distortion_params(1);
  distortion_params << distortion_param;
  Eigen::VectorXd intrinsics(4);
  intrinsics << fu, fv, cu, cv;

  // Create a vector with all cameras.
  std::vector<aslam::Camera::Ptr> camera_vector;

  // Transformation vector for all cameras.
  aslam::TransformationVector T_C_B_vector;

  // Fill the camera vector.
  for (size_t camera_idx = 0; camera_idx < kNumCams; ++camera_idx) {
    aslam::Distortion::UniquePtr distortion(
        new DistortionType(distortion_params));
    aslam::Camera::Ptr camera = std::shared_ptr<CameraType>(
        new CameraType(intrinsics, ru, rv, distortion));
    CHECK(camera);
    aslam::CameraId cam_id;
    cam_id.randomize();
    camera->setId(cam_id);
    camera_vector.push_back(camera);

    // Extrinsics
    Eigen::Matrix4d T_B_C;
    T_B_C.block<3, 3>(0, 0) = camera_rotations[camera_idx];
    T_B_C.block<3, 1>(0, 3) = camera_offsets[camera_idx];
    pose::Transformation T_B_C_(T_B_C);
    T_C_B_vector.push_back(T_B_C_.inverse());
  }

  // Create the camera rig.
  // Construct the ID object.
  aslam::NCameraId camera_rig_id;
  // Randomize the ID.
  camera_rig_id.randomize();
  aslam::NCamera::Ptr camera_rig;
  camera_rig.reset(
      new aslam::NCamera(
          camera_rig_id, T_C_B_vector, camera_vector, "pose_estimator_test"));

  // Convert the openGV simulation data to nframes to match the interface.
  std::vector<std::vector<size_t> > nframe_0_indices;
  std::vector<std::vector<size_t> > nframe_1_indices;
  aslam::VisualNFrame::Ptr nframe_0, nframe_1;
  for (size_t nframe_idx = 0; nframe_idx < kNumNFrames; ++nframe_idx) {
    aslam::NFramesId nframe_id;
    nframe_id.randomize();
    if (nframe_idx == kViewpoint0) {
      nframe_0 = aligned_shared<aslam::VisualNFrame>(nframe_id, camera_rig);
    } else if (nframe_idx == kViewpoint1) {
      nframe_1 = aligned_shared<aslam::VisualNFrame>(nframe_id, camera_rig);
    }
    // Fill in a frame for each camera in the rig.
    for (size_t camera_idx = 0; camera_idx < kNumCams; ++camera_idx) {
      // Sets up the frame with id, assigns the cameras, etc.
      aslam::VisualFrame::Ptr frame(new aslam::VisualFrame);
      aslam::FrameId frame_id;
      frame_id.randomize();
      frame->setId(frame_id);
      aslam::Camera::ConstPtr camera = camera_rig->getCameraShared(camera_idx);
      CHECK(camera);
      frame->setCameraGeometry(camera);

      // Add the measurement in nframe 0 and nframe 1.
      std::vector<Eigen::Matrix<double, 3, 1>,
                  Eigen::aligned_allocator<Eigen::Matrix<double, 3, 1> > >
          bearing_vector;
      if (nframe_idx == kViewpoint0) {
        bearing_vector = *(multi_bearing_vectors_nframe_0[camera_idx]);
      } else if (nframe_idx == kViewpoint1) {
        bearing_vector = *(multi_bearing_vectors_nframe_1[camera_idx]);
      }
      size_t num_measurements = bearing_vector.size();
      Eigen::Matrix2Xd keypoints;
      keypoints.resize(Eigen::NoChange, num_measurements);

      std::vector<size_t> keypoints_indices;
      // Project the bearing vectors in the cameras of both nframes.
      for (size_t i = 0; i < num_measurements; ++i) {
        Eigen::Vector2d projection;
        // Make sure that keypoint is visible.
        if (camera->project3(bearing_vector[i], &projection) ==
            aslam::ProjectionResult::Status::KEYPOINT_VISIBLE) {
          keypoints_indices.push_back(i);
        }
        keypoints(0, i) = projection(0);
        keypoints(1, i) = projection(1);
      }
      frame->setKeypointMeasurements(keypoints);
      if (nframe_idx == kViewpoint0) {
        nframe_0_indices.push_back(keypoints_indices);
        nframe_0->setFrame(camera_idx, frame);
      } else if (nframe_idx == kViewpoint1) {
        nframe_1_indices.push_back(keypoints_indices);
        nframe_1->setFrame(camera_idx, frame);
      }
    }  // Loop over all cameras.
  }    // Loop over both nframes.

  // Create matches pairs
  aslam::FrameToFrameMatchesList matches_paired(kNumCams);
  for (size_t camera_idx = 0; camera_idx < kNumCams; ++camera_idx) {
    // Fix index of first frame...
    for (size_t keypoint_idx_nframe_0 = 0;
         keypoint_idx_nframe_0 < nframe_0_indices[camera_idx].size();
         ++keypoint_idx_nframe_0) {
      // loop over indices in second frame ...
      for (size_t keypoint_idx_nframe_1 = 0;
           keypoint_idx_nframe_1 < nframe_1_indices[camera_idx].size();
           ++keypoint_idx_nframe_1) {
        // ... and see if it was found in the second frame.
        size_t kp_0 = nframe_0_indices[camera_idx][keypoint_idx_nframe_0];
        size_t kp_1 = nframe_1_indices[camera_idx][keypoint_idx_nframe_1];
        if (kp_0 == kp_1) {
          matches_paired[camera_idx].emplace_back(kp_0, kp_1);
        }
      }
    }
  }

  // Setup and run the PNP pose estimator.
  constexpr bool kNonlinearRefinement = false;
  geometric_vision::MultiNonCentralRelativeSacPnp::Algorithm solver =
      geometric_vision::MultiNonCentralRelativeSacPnp::Algorithm::SEVENTEENPT;
  geometric_vision::RelativeNonCentralPnp pose_estimator(
      kNonlinearRefinement, solver, kRandomSeed);
  const double pixel_sigma = sqrt(2);
  const size_t kRansacMaxIters = 100;
  pose::Transformation T_B0_B1;
  size_t num_inliers = 0;
  pose_estimator.computePinhole(
      nframe_0, nframe_1, matches_paired, pixel_sigma, kRansacMaxIters,
      &T_B0_B1, &num_inliers);

  // Compare ground truth relative pose to PNP estimates.
  EXPECT_NEAR_EIGEN(
      relative_translation_ground_truth, T_B0_B1.getPosition(), 1e-5);
  EXPECT_NEAR_EIGEN(
      relative_rotation_ground_truth, T_B0_B1.getRotationMatrix(), 1e-10);

  // Now run opengv PnP directly.
  // Remove observations that are not inside image box.
  std::vector<std::shared_ptr<opengv::bearingVectors_t> >
      multi_bearing_vectors_nframe_0_only_visible_landmarks;
  std::vector<std::shared_ptr<opengv::bearingVectors_t> >
      multi_bearing_vectors_nframe_1_only_visible_landmarks;
  for (size_t camera_idx = 0; camera_idx < kNumCams; ++camera_idx) {
    std::shared_ptr<opengv::bearingVectors_t> bearing_vectors_0(
        new opengv::bearingVectors_t());
    std::shared_ptr<opengv::bearingVectors_t> bearing_vectors_1(
        new opengv::bearingVectors_t());
    for (size_t i = 0; i < matches_paired[camera_idx].size(); ++i) {
      const size_t keypoint_idx_nframe_0 =
          matches_paired[camera_idx][i].getKeypointIndexAppleFrame();
      const size_t keypoint_idx_nframe_1 =
          matches_paired[camera_idx][i].getKeypointIndexBananaFrame();

      opengv::bearingVector_t bearing_vector_0 =
          (*multi_bearing_vectors_nframe_0[camera_idx])[keypoint_idx_nframe_0];
      opengv::bearingVector_t bearing_vector_1 =
          (*multi_bearing_vectors_nframe_1[camera_idx])[keypoint_idx_nframe_1];

      bearing_vectors_0->push_back(bearing_vector_0);
      bearing_vectors_1->push_back(bearing_vector_1);
    }
    multi_bearing_vectors_nframe_0_only_visible_landmarks.push_back(
        bearing_vectors_0);
    multi_bearing_vectors_nframe_1_only_visible_landmarks.push_back(
        bearing_vectors_1);
  }

  // Create a non-central relative multi-adapter.
  opengv::relative_pose::NoncentralRelativeMultiAdapter adapter(
      multi_bearing_vectors_nframe_0_only_visible_landmarks,
      multi_bearing_vectors_nframe_1_only_visible_landmarks, camera_offsets,
      camera_rotations);

  // Create a MultiNoncentralRelativePoseSacProblem and Ransac.
  opengv::sac::MultiRansac<geometric_vision::MultiNonCentralRelativeSacPnp>
      ransac(kRandomSeed);
  std::shared_ptr<geometric_vision::MultiNonCentralRelativeSacPnp> problem_ptr(
      new geometric_vision::MultiNonCentralRelativeSacPnp(
          adapter, geometric_vision::MultiNonCentralRelativeSacPnp::SEVENTEENPT,
          kAsCentral, kRandomSeed));

  ransac.sac_model_ = problem_ptr;
  const double focal_length = (fu + fv) / 2.0;
  ransac.threshold_ = 1.0 - cos(atan(pixel_sigma / focal_length));
  ransac.max_iterations_ = kRansacMaxIters;

  // Run the ransac experiment.
  ransac.computeModel();

  Eigen::Vector3d relative_translation_opengv =
      ransac.model_coefficients_.block<3, 1>(0, 3);
  Eigen::Matrix3d relative_rotation_opengv =
      ransac.model_coefficients_.block<3, 3>(0, 0);

  // Compare the direct opengv PNP result to PNP result from interface.
  EXPECT_NEAR_EIGEN(relative_translation_opengv, T_B0_B1.getPosition(), 1e-3);
  EXPECT_NEAR_EIGEN(
      relative_rotation_opengv, T_B0_B1.getRotationMatrix(), 1e-3);
}

// *************************************************************
// The test goes as follows:
// [1] Generate the cameras.
// [2] Generate the path and landmarks.
// [3] Setup and run the visual nframe simulator.
// [4] Feed the visual nframes to the matcher.
// [5] Convert aslam::MatchingProblemFrameToFrame::Matches to
// geometric_vision::Matches
// [6] Setup and run the PNP pose estimator.
// [7] Validate the PNP pose estimates.
// *************************************************************
TEST_F(RelativeNonCentralPnpTest, testInterfaceWithGeometricVisionSetup) {
  // **************************************************
  // [1] Generate the cameras.
  // **************************************************
  const size_t kNumCams = 4;
  aslam::NCamera::Ptr n_camera = aslam::NCamera::createTestNCamera(kNumCams);

  // **************************************************
  // [2] Generate the path and landmarks.
  // **************************************************
  test_trajectory_gen::PathAndLandmarkSettings settings;
  settings.mode = test_trajectory_gen::Path::kCircular;
  settings.imu_sigmas.acc_bias_random_walk_noise_density = 0.0;
  settings.imu_sigmas.acc_noise_density = 0.0;
  settings.gravity_meter_by_second2 = 9.81;
  settings.imu_sigmas.gyro_bias_random_walk_noise_density = 0.0;
  settings.imu_sigmas.gyro_noise_density = 0.0;
  settings.imu_noise_bias_seed = 10u;
  settings.num_of_path_constraints = 8;
  settings.landmark_seed = 5u;
  settings.landmark_variance_meter = 0.25;
  settings.num_of_landmarks = 1000;
  settings.sampling_time_second = 0.1;
  settings.distance_to_keypoints_meter = 7.5;

  std::shared_ptr<test_trajectory_gen::GenericPathGenerator> path_generator;
  path_generator =
      aligned_shared<test_trajectory_gen::GenericPathGenerator>(settings);
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
  const bool kAddNoiseToKeypoints = false;
  const size_t kNumBitsToFlip = 0u;
  const double kKeypointSigmaPixel = 0.8;

  aslam::TransformationVector T_G_Bs;
  path_generator->getGroundTruthTransformations(&T_G_Bs);
  aslam::TransformationVector poses_without_keypoints;
  nframe_simulator.simulateVisualNFrames(
      path_generator->getTimestampsInSeconds(), T_G_Bs,
      path_generator->getLandmarks(), n_camera, kKeypointSigmaPixel,
      kAddNoiseToKeypoints, kNumBitsToFlip, &nframes, &poses_without_keypoints);
  ASSERT_GT(nframes.size(), 0u) << "Zero simulated nframes. Aborting.";
  CHECK(poses_without_keypoints.empty());

  // **************************************************
  // [4] Feed the visual nframes to the matcher.
  // **************************************************
  // Choose two consecutive (arbitrary) nframes with some translation in
  // between.
  constexpr size_t kFrame0Idx = 3;
  constexpr size_t kFrame1Idx = 4;

  const aslam::Transformation& T_G_B0 = true_T_G_B[kFrame0Idx];
  const aslam::Transformation& T_G_B1 = true_T_G_B[kFrame1Idx];

  // Set matcher parameters.
  constexpr double kImageSpaceDistanceThreshold = 500.0;
  constexpr size_t kDescriptorDistanceThreshold = 5;

  aslam::MatchingEngineExclusive<aslam::MatchingProblemFrameToFrame>
      matching_engine;
  aslam::MatchingProblemFrameToFrame::MatchesList matches_0_1;
  // Start for-loop cameras.
  for (size_t camera_idx = 0; camera_idx < kNumCams; ++camera_idx) {
    // Previous frame == frame 0.
    aslam::VisualFrame::Ptr frame0 =
        nframes[kFrame0Idx]->getFrameShared(camera_idx);
    CHECK(frame0);
    CHECK_EQ(nframes[kFrame0Idx]->getNumFrames(), kNumCams);

    // Current frame == frame 1.
    aslam::VisualFrame::Ptr frame1 =
        nframes[kFrame1Idx]->getFrameShared(camera_idx);
    CHECK(frame1);
    CHECK_EQ(nframes[kFrame1Idx]->getNumFrames(), kNumCams);

    // Get the ground truth rotation between the first and second frame.
    const aslam::Transformation& T_C_B0 =
        nframes[kFrame0Idx]->get_T_C_B(camera_idx);
    const aslam::Transformation& T_C_B1 =
        nframes[kFrame1Idx]->get_T_C_B(camera_idx);
    const aslam::Transformation T_C0_C1 =
        T_C_B0 * T_G_B0.inverse() * T_G_B1 * T_C_B1.inverse();
    aslam::MatchingProblemFrameToFrame matching_problem(
        *frame0, *frame1, T_C0_C1.getRotation(), kImageSpaceDistanceThreshold,
        kDescriptorDistanceThreshold);

    aslam::MatchingProblemFrameToFrame::Matches matches;
    matching_engine.match(&matching_problem, &matches);
    matches_0_1.push_back(matches);
  }  // End for-loop cameras.

  // **************************************************
  // [5] Setup and run the PNP pose estimator.
  // **************************************************
  // Force deterministic SAC problem.
  constexpr bool kNonlinearRefinement = false;
  constexpr bool kRandomSeed = false;
  geometric_vision::MultiNonCentralRelativeSacPnp::Algorithm solver =
      geometric_vision::MultiNonCentralRelativeSacPnp::Algorithm::SEVENTEENPT;
  geometric_vision::RelativeNonCentralPnp pose_estimator(
      kNonlinearRefinement, solver, kRandomSeed);
  constexpr double kPixelSigma = 0.8;
  constexpr size_t kRansacMaxIters = 100;
  pose::Transformation pnp_T_B0_B1;
  size_t num_inliers = 0;
  pose_estimator.computePinhole(
      nframes[kFrame0Idx], nframes[kFrame1Idx], matches_0_1, kPixelSigma,
      kRansacMaxIters, &pnp_T_B0_B1, &num_inliers);

  // **************************************************
  // [6] Validate the PNP pose estimates.
  // **************************************************
  const aslam::Transformation true_T_B1_B0 =
      true_T_G_B[kFrame1Idx].inverse() * true_T_G_B[kFrame0Idx];
  const aslam::Transformation pnp_T_B1_B0 = pnp_T_B0_B1.inverse();

  EXPECT_NEAR_EIGEN(
      true_T_B1_B0.getPosition().normalized(),
      pnp_T_B1_B0.getPosition().normalized(), 1.0e-4);
  EXPECT_NEAR_EIGEN(
      true_T_B1_B0.getRotationMatrix(), pnp_T_B1_B0.getRotationMatrix(),
      1.0e-8);
}

MAPLAB_UNITTEST_ENTRYPOINT
