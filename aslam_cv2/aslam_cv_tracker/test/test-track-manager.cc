#include <aslam/cameras/camera-pinhole.h>
#include <aslam/common/entrypoint.h>
#include <aslam/frames/visual-frame.h>
#include <aslam/matcher/match.h>
#include <aslam/tracker/track-manager.h>
#include <eigen-checks/gtest.h>
#include <Eigen/Core>
#include <gtest/gtest.h>

TEST(TrackManagerTests, TestApplyMatcher) {
  aslam::TrackManager::resetIdProvider();

  aslam::Camera::Ptr camera = aslam::PinholeCamera::createTestCamera();
  aslam::VisualFrame::Ptr B_frame =
      aslam::VisualFrame::createEmptyTestVisualFrame(camera, 0);
  aslam::VisualFrame::Ptr A_frame =
      aslam::VisualFrame::createEmptyTestVisualFrame(camera, 1);

  Eigen::Matrix2Xd B_keypoints = Eigen::Matrix2Xd::Zero(2, 5);
  Eigen::Matrix2Xd A_keypoints = Eigen::Matrix2Xd::Zero(2, 5);

  // B frame: -1, -1, 0, 1, -1
  // A frame:  -1, 2, -1, -1, -1
  Eigen::VectorXi B_tracks(5);
  B_tracks << -1, -1, 0, 1, -1;

  Eigen::VectorXi A_tracks(5);
  A_tracks << -1, 2, -1, -1, -1;

  B_frame->swapKeypointMeasurements(&B_keypoints);
  A_frame->swapKeypointMeasurements(&A_keypoints);

  B_frame->swapTrackIds(&B_tracks);
  A_frame->swapTrackIds(&A_tracks);

  // matches_A_B: {(0,0), (1,1), (2,2), (3,3), (4,4)}
  aslam::FrameToFrameMatches matches_A_B;
  matches_A_B.reserve(5);

  matches_A_B.emplace_back(0, 0);
  matches_A_B.emplace_back(1, 1);
  matches_A_B.emplace_back(2, 2);
  matches_A_B.emplace_back(3, 3);
  matches_A_B.emplace_back(4, 4);

  aslam::SimpleTrackManager track_manager;
  track_manager.applyMatchesToFrames(matches_A_B, A_frame.get(), B_frame.get());

  // Expected output:
  // B frame: 3, 2, 0, 1, 4
  // A frame:  3, 2, 0, 1, 4
  Eigen::VectorXi expected_B_tracks(5);
  expected_B_tracks << 0, 2, 0, 1, 1;

  Eigen::VectorXi expected_A_tracks(5);
  expected_A_tracks << 0, 2, 0, 1, 1;

  B_tracks = B_frame->getTrackIds();
  A_tracks = A_frame->getTrackIds();

  EXPECT_TRUE(EIGEN_MATRIX_EQUAL(expected_B_tracks, B_tracks));
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL(expected_A_tracks, A_tracks));
}

TEST(TrackManagerTests, TestApplyMatchesEmpty) {
  aslam::TrackManager::resetIdProvider();

  aslam::Camera::Ptr camera = aslam::PinholeCamera::createTestCamera();
  aslam::VisualFrame::Ptr B_frame =
      aslam::VisualFrame::createEmptyTestVisualFrame(camera, 0);
  aslam::VisualFrame::Ptr A_frame =
      aslam::VisualFrame::createEmptyTestVisualFrame(camera, 1);

  Eigen::Matrix2Xd B_keypoints = Eigen::Matrix2Xd::Zero(2, 5);
  Eigen::Matrix2Xd A_keypoints = Eigen::Matrix2Xd::Zero(2, 5);

  Eigen::VectorXi B_tracks = Eigen::VectorXi::Constant(5, -1);
  Eigen::VectorXi A_tracks = Eigen::VectorXi::Constant(5, -1);

  B_frame->swapKeypointMeasurements(&B_keypoints);
  A_frame->swapKeypointMeasurements(&A_keypoints);

  B_frame->swapTrackIds(&B_tracks);
  A_frame->swapTrackIds(&A_tracks);

  aslam::FrameToFrameMatches matches_A_B;

  aslam::SimpleTrackManager track_manager;
  track_manager.applyMatchesToFrames(matches_A_B, A_frame.get(), B_frame.get());

  // Expected output:
  Eigen::VectorXi expected_B_tracks = Eigen::VectorXi::Constant(5, -1);
  Eigen::VectorXi expected_A_tracks = Eigen::VectorXi::Constant(5, -1);

  B_tracks = B_frame->getTrackIds();
  A_tracks = A_frame->getTrackIds();

  EXPECT_TRUE(EIGEN_MATRIX_EQUAL(expected_B_tracks, B_tracks));
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL(expected_A_tracks, A_tracks));
}

ASLAM_UNITTEST_ENTRYPOINT
