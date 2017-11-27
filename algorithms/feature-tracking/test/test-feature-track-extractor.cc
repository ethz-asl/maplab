#include <unordered_set>
#include <vector>

#include <Eigen/Dense>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/camera.h>
#include <aslam/cameras/ncamera.h>
#include <aslam/frames/feature-track.h>
#include <aslam/frames/visual-frame.h>
#include <aslam/frames/visual-nframe.h>
#include <maplab-common/test/testing-entrypoint.h>

#include "feature-tracking/feature-track-extractor.h"

TEST(FeatureTrackExtractor, TestExtractionFromStream) {
  // Create 7 test frames so that we have the following tracks:
  //             F0    F1    F2    F3    F4    F5    F6
  //  Track 0    *------*-----*
  //  Track 1           *-----*----*-----*------*
  //  Untracked  *      *          *            *     *
  //  Track 2    *------*
  //  Track 3    *------*
  //  Track 4                      *-----*------*
  //  Track 5                            *------*
  //  Untracked  *            *          *
  aslam::NCamera::Ptr ncamera = aslam::NCamera::createTestNCamera(1);
  std::vector<aslam::VisualNFrame::Ptr> nframes;
  for (int c = 0; c < 7; ++c) {
    nframes.emplace_back(
        aslam::VisualNFrame::createEmptyTestVisualNFrame(ncamera, c));
  }

  Eigen::VectorXi track_ids;
  Eigen::Matrix2Xd keypoints;

  // Frame 0
  track_ids.resize(5);
  track_ids << 0, -1, 2, 3, -1;
  keypoints.resize(Eigen::NoChange, 5);
  keypoints.setOnes();
  keypoints *= 0;
  nframes[0]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[0]->getFrameShared(0)->setKeypointMeasurements(keypoints);
  // Frame 1
  track_ids.resize(5);
  track_ids << 0, 1, -1, 2, 3;
  keypoints.resize(Eigen::NoChange, 5);
  keypoints.setOnes();
  keypoints *= 1;
  nframes[1]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[1]->getFrameShared(0)->setKeypointMeasurements(keypoints);
  // Frame 2
  track_ids.resize(3);
  track_ids << 0, 1, -1;
  keypoints.resize(Eigen::NoChange, 3);
  keypoints.setOnes();
  keypoints *= 2;
  nframes[2]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[2]->getFrameShared(0)->setKeypointMeasurements(keypoints);
  // Frame 3
  track_ids.resize(3);
  track_ids << 1, -1, 4;
  keypoints.resize(Eigen::NoChange, 3);
  keypoints.setOnes();
  keypoints *= 3;
  nframes[3]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[3]->getFrameShared(0)->setKeypointMeasurements(keypoints);
  // Frame 4
  track_ids.resize(4);
  track_ids << 1, 4, 5, -1;
  keypoints.resize(Eigen::NoChange, 4);
  keypoints.setOnes();
  keypoints *= 4;
  nframes[4]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[4]->getFrameShared(0)->setKeypointMeasurements(keypoints);
  // Frame 5
  track_ids.resize(4);
  track_ids << 1, -1, 4, 5;
  keypoints.resize(Eigen::NoChange, 4);
  keypoints.setOnes();
  keypoints *= 5;
  nframes[5]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[5]->getFrameShared(0)->setKeypointMeasurements(keypoints);
  // Frame 6
  track_ids.resize(1);
  track_ids << -1;
  keypoints.resize(Eigen::NoChange, 1);
  keypoints.setOnes();
  keypoints *= 6;
  nframes[6]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[6]->getFrameShared(0)->setKeypointMeasurements(keypoints);

  // Ground truth data: which tracks should end at which frame
  // track_ids_groundtruth --> { {}, {}, {2, 3}, {0}, {}, {}, {1, 4, 5} };
  std::vector<std::set<int>> track_ids_groundtruth;
  track_ids_groundtruth.emplace_back(std::set<int>());
  track_ids_groundtruth.emplace_back(std::set<int>());
  std::set<int> track0;
  track0.insert(2);
  track0.insert(3);
  track_ids_groundtruth.emplace_back(track0);
  std::set<int> track1;
  track1.insert(0);
  track_ids_groundtruth.emplace_back(track1);
  track_ids_groundtruth.emplace_back(std::set<int>());
  track_ids_groundtruth.emplace_back(std::set<int>());
  std::set<int> track2;
  track2.insert(1);
  track2.insert(4);
  track2.insert(5);
  track_ids_groundtruth.emplace_back(track2);

  // Ground truth track lengths.
  std::vector<size_t> ground_truth_track_lengths(6, 0);
  ground_truth_track_lengths[0] = 3u;
  ground_truth_track_lengths[1] = 5u;
  ground_truth_track_lengths[2] = 2u;
  ground_truth_track_lengths[3] = 2u;
  ground_truth_track_lengths[4] = 3u;
  ground_truth_track_lengths[5] = 2u;

  // Extract the tracks.
  aslam::FeatureTracksList tracks_at_time;
  tracks_at_time.resize(7);
  const size_t kMaxTrackLength = 100;
  const size_t kMinTrackLength = 1;

  vio_common::FeatureTrackExtractor extractor(
      ncamera, kMaxTrackLength, kMinTrackLength);
  for (int i = 0; i < 7; ++i) {
    aslam::FeatureTracksList all_tracks;
    size_t num_tracks =
        extractor.extractFromNFrameStream(nframes[i], &all_tracks);
    ASSERT_EQ(all_tracks.size(), ncamera->getNumCameras());
    const size_t kCamera0Idx = 0u;
    tracks_at_time[i] = all_tracks[kCamera0Idx];

    EXPECT_EQ(track_ids_groundtruth[i].size(), num_tracks);
    VLOG(3) << "Got " << num_tracks << " finished track(s) at step " << i
            << ".";
    for (const auto& track : tracks_at_time[i]) {
      EXPECT_GT(track_ids_groundtruth[i].count(track.getTrackId()), 0u);
      EXPECT_GE(track.getTrackId(), 0u);
      EXPECT_EQ(
          ground_truth_track_lengths[track.getTrackId()],
          track.getTrackLength());
    }
  }
}

TEST(FeatureTrackExtractor, TestExtractionFromStreamWithMinTrackLength) {
  // Create 7 test frames so that we have the following tracks:
  //             F0    F1    F2    F3    F4    F5    F6
  //  Track 0    *------*-----*
  //  Track 1           *-----*----*-----*------*
  //  Untracked  *      *          *            *     *
  //  Track 2    *------*
  //  Track 3    *------*
  //  Track 4                      *-----*------*
  //  Track 5                            *------*
  //  Untracked  *            *          *
  size_t kMinTrackLength = 3u;
  // Expecting track 0, 1, 4 to terminate.

  aslam::NCamera::Ptr ncamera = aslam::NCamera::createTestNCamera(1);
  std::vector<aslam::VisualNFrame::Ptr> nframes;
  for (int c = 0; c < 7; ++c) {
    nframes.emplace_back(
        aslam::VisualNFrame::createEmptyTestVisualNFrame(ncamera, c));
  }

  Eigen::VectorXi track_ids;
  Eigen::Matrix2Xd keypoints;

  // Frame 0
  track_ids.resize(5);
  track_ids << 0, -1, 2, 3, -1;
  keypoints.resize(Eigen::NoChange, 5);
  keypoints.setOnes();
  keypoints *= 0;
  nframes[0]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[0]->getFrameShared(0)->setKeypointMeasurements(keypoints);
  // Frame 1
  track_ids.resize(5);
  track_ids << 0, 1, -1, 2, 3;
  keypoints.resize(Eigen::NoChange, 5);
  keypoints.setOnes();
  keypoints *= 1;
  nframes[1]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[1]->getFrameShared(0)->setKeypointMeasurements(keypoints);
  // Frame 2
  track_ids.resize(3);
  track_ids << 0, 1, -1;
  keypoints.resize(Eigen::NoChange, 3);
  keypoints.setOnes();
  keypoints *= 2;
  nframes[2]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[2]->getFrameShared(0)->setKeypointMeasurements(keypoints);
  // Frame 3
  track_ids.resize(3);
  track_ids << 1, -1, 4;
  keypoints.resize(Eigen::NoChange, 3);
  keypoints.setOnes();
  keypoints *= 3;
  nframes[3]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[3]->getFrameShared(0)->setKeypointMeasurements(keypoints);
  // Frame 4
  track_ids.resize(4);
  track_ids << 1, 4, 5, -1;
  keypoints.resize(Eigen::NoChange, 4);
  keypoints.setOnes();
  keypoints *= 4;
  nframes[4]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[4]->getFrameShared(0)->setKeypointMeasurements(keypoints);
  // Frame 5
  track_ids.resize(4);
  track_ids << 1, -1, 4, 5;
  keypoints.resize(Eigen::NoChange, 4);
  keypoints.setOnes();
  keypoints *= 5;
  nframes[5]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[5]->getFrameShared(0)->setKeypointMeasurements(keypoints);
  // Frame 6
  track_ids.resize(1);
  track_ids << -1;
  keypoints.resize(Eigen::NoChange, 1);
  keypoints.setOnes();
  keypoints *= 6;
  nframes[6]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[6]->getFrameShared(0)->setKeypointMeasurements(keypoints);

  // Ground truth data: which tracks should end at which frame
  // track_ids_groundtruth --> { {}, {}, {}, {0}, {}, {}, {1, 4} };
  std::vector<std::set<int>> track_ids_groundtruth;
  track_ids_groundtruth.emplace_back(std::set<int>());
  track_ids_groundtruth.emplace_back(std::set<int>());
  track_ids_groundtruth.emplace_back(std::set<int>());
  std::set<int> track1;
  track1.insert(0);
  track_ids_groundtruth.emplace_back(track1);
  track_ids_groundtruth.emplace_back(std::set<int>());
  track_ids_groundtruth.emplace_back(std::set<int>());
  std::set<int> track2;
  track2.insert(1);
  track2.insert(4);
  track_ids_groundtruth.emplace_back(track2);

  // Ground truth track lengths.
  std::vector<size_t> ground_truth_track_lengths(6, 0);
  ground_truth_track_lengths[0] = 3u;
  ground_truth_track_lengths[1] = 5u;
  ground_truth_track_lengths[2] = 2u;
  ground_truth_track_lengths[3] = 2u;
  ground_truth_track_lengths[4] = 3u;
  ground_truth_track_lengths[5] = 2u;

  // Extract the tracks
  aslam::FeatureTracksList tracks_at_time;
  tracks_at_time.resize(7);
  const size_t kMaxTrackLength = 100;

  vio_common::FeatureTrackExtractor extractor(
      ncamera, kMaxTrackLength, kMinTrackLength);
  for (int i = 0; i < 7; ++i) {
    aslam::FeatureTracksList all_tracks;
    size_t num_tracks =
        extractor.extractFromNFrameStream(nframes[i], &all_tracks);
    ASSERT_EQ(all_tracks.size(), ncamera->getNumCameras());
    const size_t kCamera0Idx = 0u;
    tracks_at_time[i] = all_tracks[kCamera0Idx];

    EXPECT_EQ(track_ids_groundtruth[i].size(), num_tracks);
    VLOG(3) << "Got " << num_tracks << " finished track(s) at step " << i
            << ".";
    for (const auto& track : tracks_at_time[i]) {
      EXPECT_GT(track_ids_groundtruth[i].count(track.getTrackId()), 0u);
      EXPECT_GE(track.getTrackId(), 0u);
      EXPECT_EQ(
          ground_truth_track_lengths[track.getTrackId()],
          track.getTrackLength());
    }
  }
}

void sortFeatureTracks(aslam::FeatureTracks* feature_tracks) {
  CHECK_NOTNULL(feature_tracks);

  std::map<int, aslam::FeatureTrack> track_id_to_feature_track_map;
  for (auto it = feature_tracks->begin(); it != feature_tracks->end(); ++it) {
    track_id_to_feature_track_map.insert(std::make_pair(it->getTrackId(), *it));
  }

  feature_tracks->clear();
  feature_tracks->reserve(track_id_to_feature_track_map.size());
  for (auto it = track_id_to_feature_track_map.begin();
       it != track_id_to_feature_track_map.end(); ++it) {
    feature_tracks->push_back(it->second);
  }
}

TEST(FeatureTrackExtractor, TestFullTrackInitialization) {
  // Create 7 test frames so that we have the following tracks:
  //             F0    F1    F2    F3    F4    F5    F6
  //  Track 0    *------*-----*
  //  Track 1           *-----*----*--X--*------*
  //  Untracked  *      *          *            *     *
  //  Track 2    *------*          *-----*------*
  //  Track 3    *------*                *------*
  //  Untracked  *            *          *
  const size_t kCamera0Idx = 0u;
  aslam::NCamera::Ptr ncamera = aslam::NCamera::createTestNCamera(1);
  std::vector<aslam::VisualNFrame::Ptr> nframes;
  for (int c = 0; c < 7; ++c) {
    nframes.emplace_back(
        aslam::VisualNFrame::createEmptyTestVisualNFrame(ncamera, c));
  }

  Eigen::VectorXi track_ids;
  Eigen::Matrix2Xd keypoints;

  // Fill frame 0 with no tracks.
  track_ids.resize(5);
  track_ids.setConstant(-1);
  keypoints.resize(Eigen::NoChange, 5);
  keypoints.setZero();
  nframes[0]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[0]->getFrameShared(0)->setKeypointMeasurements(keypoints);

  aslam::FeatureTracksList tracks_at_time;
  tracks_at_time.resize(7);
  const size_t kMaxTrackLength = 3;
  const size_t kMinTrackLength = 1;
  // Add frame 0 to the track extractor
  aslam::FeatureTracksList all_tracks;
  vio_common::FeatureTrackExtractor extractor(
      ncamera, kMaxTrackLength, kMinTrackLength);
  size_t num_tracks =
      extractor.extractFromNFrameStream(nframes[0], &all_tracks);
  ASSERT_EQ(all_tracks.size(), ncamera->getNumCameras());
  tracks_at_time[0] = all_tracks[kCamera0Idx];

  EXPECT_EQ(num_tracks, 0u);
  EXPECT_TRUE(tracks_at_time[0].empty());

  // Add matching results to frame 0 and frame 1.

  // Frame 0
  track_ids.resize(5);
  track_ids << 0, -1, 2, 3, -1;
  keypoints.resize(Eigen::NoChange, 5);
  keypoints.setZero();
  nframes[0]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[0]->getFrameShared(0)->setKeypointMeasurements(keypoints);
  // Frame 1
  track_ids.resize(5);
  track_ids << 0, -1, -1, 2, 3;
  keypoints.resize(Eigen::NoChange, 5);
  keypoints.setOnes();
  nframes[1]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[1]->getFrameShared(0)->setKeypointMeasurements(keypoints);

  // Add frame 1 to the extractor.
  num_tracks = extractor.extractFromNFrameStream(nframes[1], &all_tracks);
  ASSERT_EQ(all_tracks.size(), ncamera->getNumCameras());
  tracks_at_time[1] = all_tracks[kCamera0Idx];

  EXPECT_EQ(num_tracks, 0u);  // No terminated tracks yet.
  EXPECT_TRUE(tracks_at_time[1].empty());

  // Frame 1
  track_ids.resize(5);
  track_ids << 0, 1, -1, 2, 3;
  keypoints.resize(Eigen::NoChange, 5);
  keypoints.setOnes();
  nframes[1]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[1]->getFrameShared(0)->setKeypointMeasurements(keypoints);

  // Frame 2
  track_ids.resize(3);
  track_ids << 0, 1, -1;
  keypoints.resize(Eigen::NoChange, 3);
  keypoints.setOnes();
  keypoints *= 2;
  nframes[2]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[2]->getFrameShared(0)->setKeypointMeasurements(keypoints);

  // Add frame 2 to the extractor.
  num_tracks = extractor.extractFromNFrameStream(nframes[2], &all_tracks);
  ASSERT_EQ(all_tracks.size(), ncamera->getNumCameras());
  tracks_at_time[2] = all_tracks[kCamera0Idx];

  // The following tracks are expected to terminate:
  // 2 -> length 2
  // 3 -> length 2
  EXPECT_EQ(num_tracks, 2u);
  sortFeatureTracks(&tracks_at_time[2]);

  EXPECT_EQ(tracks_at_time[2].size(), 2u);
  const aslam::FeatureTrack& track_2 = tracks_at_time[2][0];
  const aslam::FeatureTrack& track_3 = tracks_at_time[2][1];

  EXPECT_EQ(track_2.getTrackLength(), 2u);
  EXPECT_EQ(track_3.getTrackLength(), 2u);

  EXPECT_EQ(track_2.getTrackId(), 2u);
  EXPECT_EQ(track_3.getTrackId(), 3u);

  // Frame 2
  track_ids.resize(3);
  track_ids << 0, 1, -1;
  keypoints.resize(Eigen::NoChange, 3);
  keypoints.setOnes();
  keypoints *= 2;
  nframes[2]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[2]->getFrameShared(0)->setKeypointMeasurements(keypoints);

  // Frame 3
  track_ids.resize(3);
  track_ids << 1, -1, -1;
  keypoints.resize(Eigen::NoChange, 3);
  keypoints.setOnes();
  keypoints *= 3;
  nframes[3]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[3]->getFrameShared(0)->setKeypointMeasurements(keypoints);

  // Add frame 3 to the extractor.
  num_tracks = extractor.extractFromNFrameStream(nframes[3], &all_tracks);
  ASSERT_EQ(all_tracks.size(), ncamera->getNumCameras());
  tracks_at_time[3] = all_tracks[kCamera0Idx];

  // We expect the following tracks to terminate:
  // 0 -> length 3
  EXPECT_EQ(num_tracks, 1u);
  EXPECT_EQ(tracks_at_time[3].size(), 1u);

  const aslam::FeatureTrack& track_0 = tracks_at_time[3][0];
  EXPECT_EQ(track_0.getTrackLength(), 3u);
  EXPECT_EQ(track_0.getTrackId(), 0u);

  // Frame 3
  track_ids.resize(3);
  track_ids << 1, -1, 2;
  keypoints.resize(Eigen::NoChange, 3);
  keypoints.setOnes();
  keypoints *= 3;
  nframes[3]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[3]->getFrameShared(0)->setKeypointMeasurements(keypoints);

  // Frame 4
  track_ids.resize(4);
  track_ids << 1, 2, -1, -1;
  keypoints.resize(Eigen::NoChange, 4);
  keypoints.setOnes();
  keypoints *= 4;
  nframes[4]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[4]->getFrameShared(0)->setKeypointMeasurements(keypoints);

  // Add frame 4 to the extractor.
  num_tracks = extractor.extractFromNFrameStream(nframes[4], &all_tracks);
  ASSERT_EQ(all_tracks.size(), ncamera->getNumCameras());
  tracks_at_time[4] = all_tracks[kCamera0Idx];

  // We expect the following tracks to terminate:
  // 1 -> length 3 (because reaching max length)
  EXPECT_EQ(num_tracks, 1u);
  EXPECT_EQ(tracks_at_time[4].size(), 1u);

  const aslam::FeatureTrack& track_1 = tracks_at_time[4][0];
  EXPECT_EQ(track_1.getTrackLength(), 3u);
  EXPECT_EQ(track_1.getTrackId(), 1u);

  // Frame 4
  track_ids.resize(4);
  track_ids << 1, 2, 3, -1;
  keypoints.resize(Eigen::NoChange, 4);
  keypoints.setOnes();
  keypoints *= 4;
  nframes[4]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[4]->getFrameShared(0)->setKeypointMeasurements(keypoints);

  // Frame 5
  track_ids.resize(4);
  track_ids << 1, -1, 2, 3;
  keypoints.resize(Eigen::NoChange, 4);
  keypoints.setOnes();
  keypoints *= 5;
  nframes[5]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[5]->getFrameShared(0)->setKeypointMeasurements(keypoints);

  // Add frame 5 to the extractor.
  num_tracks = extractor.extractFromNFrameStream(nframes[5], &all_tracks);
  ASSERT_EQ(all_tracks.size(), ncamera->getNumCameras());
  tracks_at_time[5] = all_tracks[kCamera0Idx];

  // We expect no tracks to terminate in this frame.
  EXPECT_EQ(num_tracks, 0u);  // No terminated tracks yet.
  EXPECT_TRUE(tracks_at_time[5].empty());

  // Frame 5 remains unchanged.

  // Frame 6
  track_ids.resize(1);
  track_ids << -1;
  keypoints.resize(Eigen::NoChange, 1);
  keypoints.setOnes();
  keypoints *= 6;
  nframes[6]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[6]->getFrameShared(0)->setKeypointMeasurements(keypoints);

  // Add frame 6 to the extractor.
  num_tracks = extractor.extractFromNFrameStream(nframes[6], &all_tracks);
  ASSERT_EQ(all_tracks.size(), ncamera->getNumCameras());
  tracks_at_time[6] = all_tracks[kCamera0Idx];

  // We expect the following tracks to terminate:
  // 1 -> length 2
  // 2 -> length 3
  // 3 -> length 2

  EXPECT_EQ(num_tracks, 3u);
  sortFeatureTracks(&tracks_at_time[6]);
  EXPECT_EQ(tracks_at_time[6].size(), 3u);

  const aslam::FeatureTrack& track_11 = tracks_at_time[6][0];
  const aslam::FeatureTrack& track_22 = tracks_at_time[6][1];
  const aslam::FeatureTrack& track_33 = tracks_at_time[6][2];

  EXPECT_EQ(track_11.getTrackLength(), 2u);
  EXPECT_EQ(track_11.getTrackId(), 1u);

  EXPECT_EQ(track_22.getTrackLength(), 3u);
  EXPECT_EQ(track_22.getTrackId(), 2u);

  EXPECT_EQ(track_33.getTrackLength(), 2u);
  EXPECT_EQ(track_33.getTrackId(), 3u);
}

TEST(FeatureTrackExtractor, TestFullTrackInitialization_MinMaxTrackLength) {
  // Create 7 test frames so that we have the following tracks:
  //             F0    F1    F2     F3    F4   F5    F6
  //  Track 0    *------*--x--*
  //  Track 1           *-----*--x--*-----*--x--*
  //  Untracked  *      *           *           *     *
  //  Track 2    *------*           *-----*--x--*
  //  Track 3    *------*                 *-----*
  //  Untracked  *            *           *

  // Max track length: 2
  const size_t kCamera0Idx = 0u;
  aslam::NCamera::Ptr ncamera = aslam::NCamera::createTestNCamera(1);
  std::vector<aslam::VisualNFrame::Ptr> nframes;
  for (int c = 0; c < 7; ++c) {
    nframes.emplace_back(
        aslam::VisualNFrame::createEmptyTestVisualNFrame(ncamera, c));
  }

  Eigen::VectorXi track_ids;
  Eigen::Matrix2Xd keypoints;

  // Fill frame 0 with no tracks.
  track_ids.resize(5);
  track_ids.setConstant(-1);
  keypoints.resize(Eigen::NoChange, 5);
  keypoints.setZero();
  nframes[0]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[0]->getFrameShared(0)->setKeypointMeasurements(keypoints);

  aslam::FeatureTracksList tracks_at_time;
  tracks_at_time.resize(7);
  const size_t kMaxTrackLength = 2;
  const size_t kMinTrackLength = 1;
  // Add frame 0 to the track extractor
  aslam::FeatureTracksList all_tracks;
  vio_common::FeatureTrackExtractor extractor(
      ncamera, kMaxTrackLength, kMinTrackLength);
  size_t num_tracks =
      extractor.extractFromNFrameStream(nframes[0], &all_tracks);
  ASSERT_EQ(all_tracks.size(), ncamera->getNumCameras());
  tracks_at_time[0] = all_tracks[kCamera0Idx];

  EXPECT_EQ(num_tracks, 0u);
  EXPECT_TRUE(tracks_at_time[0].empty());

  // Add matching results to frame 0 and frame 1.

  // Frame 0
  track_ids.resize(5);
  track_ids << 0, -1, 2, 3, -1;
  keypoints.resize(Eigen::NoChange, 5);
  keypoints.setZero();
  nframes[0]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[0]->getFrameShared(0)->setKeypointMeasurements(keypoints);
  // Frame 1
  track_ids.resize(5);
  track_ids << 0, -1, -1, 2, 3;
  keypoints.resize(Eigen::NoChange, 5);
  keypoints.setOnes();
  nframes[1]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[1]->getFrameShared(0)->setKeypointMeasurements(keypoints);

  // Add frame 1 to the extractor.
  num_tracks = extractor.extractFromNFrameStream(nframes[1], &all_tracks);
  ASSERT_EQ(all_tracks.size(), ncamera->getNumCameras());
  tracks_at_time[1] = all_tracks[kCamera0Idx];

  EXPECT_EQ(num_tracks, 0u);  // No terminated tracks yet.
  EXPECT_TRUE(tracks_at_time[1].empty());

  // Frame 1
  track_ids.resize(5);
  track_ids << 0, 1, -1, 2, 3;
  keypoints.resize(Eigen::NoChange, 5);
  keypoints.setOnes();
  nframes[1]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[1]->getFrameShared(0)->setKeypointMeasurements(keypoints);

  // Frame 2
  track_ids.resize(3);
  track_ids << 0, 1, -1;
  keypoints.resize(Eigen::NoChange, 3);
  keypoints.setOnes();
  keypoints *= 2;
  nframes[2]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[2]->getFrameShared(0)->setKeypointMeasurements(keypoints);

  // Add frame 2 to the extractor.
  num_tracks = extractor.extractFromNFrameStream(nframes[2], &all_tracks);
  ASSERT_EQ(all_tracks.size(), ncamera->getNumCameras());
  tracks_at_time[2] = all_tracks[kCamera0Idx];

  // The following tracks are expected to terminate:
  // 0 -> length 2 (because reaching max length)
  // 2 -> length 2
  // 3 -> length 2
  EXPECT_EQ(num_tracks, 3u);
  EXPECT_EQ(tracks_at_time[2].size(), 3u);
  sortFeatureTracks(&tracks_at_time[2]);

  const aslam::FeatureTrack& track_0 = tracks_at_time[2][0];
  const aslam::FeatureTrack& track_2 = tracks_at_time[2][1];
  const aslam::FeatureTrack& track_3 = tracks_at_time[2][2];

  EXPECT_EQ(track_0.getTrackLength(), 2u);
  EXPECT_EQ(track_2.getTrackLength(), 2u);
  EXPECT_EQ(track_3.getTrackLength(), 2u);

  EXPECT_EQ(track_0.getTrackId(), 0u);
  EXPECT_EQ(track_2.getTrackId(), 2u);
  EXPECT_EQ(track_3.getTrackId(), 3u);

  // This check is to make sure the current frame is not part of any returned
  // track.
  for (auto it_track = tracks_at_time[2].begin();
       it_track != tracks_at_time[2].end(); ++it_track) {
    for (auto it_kid = it_track->getKeypointIdentifiers().begin();
         it_kid != it_track->getKeypointIdentifiers().end(); ++it_kid) {
      EXPECT_FALSE(
          it_kid->getFrameId() == nframes[2]->getFrameShared(0)->getId());
    }
  }

  // Frame 2
  track_ids.resize(3);
  track_ids << 0, 1, -1;
  keypoints.resize(Eigen::NoChange, 3);
  keypoints.setOnes();
  keypoints *= 2;
  nframes[2]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[2]->getFrameShared(0)->setKeypointMeasurements(keypoints);

  // Frame 3
  track_ids.resize(3);
  track_ids << 1, -1, -1;
  keypoints.resize(Eigen::NoChange, 3);
  keypoints.setOnes();
  keypoints *= 3;
  nframes[3]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[3]->getFrameShared(0)->setKeypointMeasurements(keypoints);

  // Add frame 3 to the extractor.
  num_tracks = extractor.extractFromNFrameStream(nframes[3], &all_tracks);
  ASSERT_EQ(all_tracks.size(), ncamera->getNumCameras());
  tracks_at_time[3] = all_tracks[kCamera0Idx];

  // We expect the following tracks to terminate:
  // 0 -> length 1
  // 1 -> length 2 (because reaching max length)
  EXPECT_EQ(num_tracks, 2u);
  EXPECT_EQ(tracks_at_time[3].size(), 2u);
  sortFeatureTracks(&tracks_at_time[3]);

  const aslam::FeatureTrack& track_00 = tracks_at_time[3][0];
  EXPECT_EQ(track_00.getTrackLength(), 1u);
  EXPECT_EQ(track_00.getTrackId(), 0u);

  const aslam::FeatureTrack& track_1 = tracks_at_time[3][1];
  EXPECT_EQ(track_1.getTrackLength(), 2u);
  EXPECT_EQ(track_1.getTrackId(), 1u);

  // This checks to make sure the current frame is not part of any returned
  // track.
  for (auto it_track = tracks_at_time[3].begin();
       it_track != tracks_at_time[3].end(); ++it_track) {
    for (auto it_kid = it_track->getKeypointIdentifiers().begin();
         it_kid != it_track->getKeypointIdentifiers().end(); ++it_kid) {
      EXPECT_FALSE(
          it_kid->getFrameId() == nframes[3]->getFrameShared(0)->getId());
    }
  }

  // Frame 3
  track_ids.resize(3);
  track_ids << 1, -1, 2;
  keypoints.resize(Eigen::NoChange, 3);
  keypoints.setOnes();
  keypoints *= 3;
  nframes[3]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[3]->getFrameShared(0)->setKeypointMeasurements(keypoints);

  // Frame 4
  track_ids.resize(4);
  track_ids << 1, 2, -1, -1;
  keypoints.resize(Eigen::NoChange, 4);
  keypoints.setOnes();
  keypoints *= 4;
  nframes[4]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[4]->getFrameShared(0)->setKeypointMeasurements(keypoints);

  // Add frame 4 to the extractor.
  num_tracks = extractor.extractFromNFrameStream(nframes[4], &all_tracks);
  ASSERT_EQ(all_tracks.size(), ncamera->getNumCameras());
  tracks_at_time[4] = all_tracks[kCamera0Idx];

  // We expect no tracks to terminate:
  EXPECT_EQ(num_tracks, 0u);
  EXPECT_TRUE(tracks_at_time[4].empty());

  // Frame 4
  track_ids.resize(4);
  track_ids << 1, 2, 3, -1;
  keypoints.resize(Eigen::NoChange, 4);
  keypoints.setOnes();
  keypoints *= 4;
  nframes[4]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[4]->getFrameShared(0)->setKeypointMeasurements(keypoints);

  // Frame 5
  track_ids.resize(4);
  track_ids << 1, -1, 2, 3;
  keypoints.resize(Eigen::NoChange, 4);
  keypoints.setOnes();
  keypoints *= 5;
  nframes[5]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[5]->getFrameShared(0)->setKeypointMeasurements(keypoints);

  // Add frame 5 to the extractor.
  num_tracks = extractor.extractFromNFrameStream(nframes[5], &all_tracks);
  ASSERT_EQ(all_tracks.size(), ncamera->getNumCameras());
  tracks_at_time[5] = all_tracks[kCamera0Idx];

  // The following tracks are expected to terminate:
  // 1 -> length 2 (because reaching max length)
  // 2 -> length 2 (because reaching max length)
  EXPECT_EQ(num_tracks, 2u);
  EXPECT_EQ(tracks_at_time[5].size(), 2u);
  sortFeatureTracks(&tracks_at_time[5]);

  EXPECT_EQ(tracks_at_time[5].size(), 2u);
  const aslam::FeatureTrack& track_11 = tracks_at_time[5][0];
  const aslam::FeatureTrack& track_22 = tracks_at_time[5][1];

  EXPECT_EQ(track_11.getTrackLength(), 2u);
  EXPECT_EQ(track_11.getTrackId(), 1u);

  EXPECT_EQ(track_22.getTrackLength(), 2u);
  EXPECT_EQ(track_22.getTrackId(), 2u);

  // This checks to make sure the current frame is not part of any returned
  // track.
  for (auto it_track = tracks_at_time[5].begin();
       it_track != tracks_at_time[5].end(); ++it_track) {
    for (auto it_kid = it_track->getKeypointIdentifiers().begin();
         it_kid != it_track->getKeypointIdentifiers().end(); ++it_kid) {
      EXPECT_FALSE(
          it_kid->getFrameId() == nframes[5]->getFrameShared(0)->getId());
    }
  }

  // Frame 5 remains unchanged.

  // Frame 6
  track_ids.resize(1);
  track_ids << -1;
  keypoints.resize(Eigen::NoChange, 1);
  keypoints.setOnes();
  keypoints *= 6;
  nframes[6]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[6]->getFrameShared(0)->setKeypointMeasurements(keypoints);

  // Add frame 6 to the extractor.
  num_tracks = extractor.extractFromNFrameStream(nframes[6], &all_tracks);
  ASSERT_EQ(all_tracks.size(), ncamera->getNumCameras());
  tracks_at_time[6] = all_tracks[kCamera0Idx];

  // We expect the following tracks to terminate:
  // 1 -> length 1
  // 2 -> length 1
  // 3 -> length 2

  EXPECT_EQ(num_tracks, 3u);
  EXPECT_EQ(tracks_at_time[6].size(), 3u);
  sortFeatureTracks(&tracks_at_time[6]);

  const aslam::FeatureTrack& track_111 = tracks_at_time[6][0];
  const aslam::FeatureTrack& track_222 = tracks_at_time[6][1];
  const aslam::FeatureTrack& track_33 = tracks_at_time[6][2];

  EXPECT_EQ(track_111.getTrackLength(), 1u);
  EXPECT_EQ(track_111.getTrackId(), 1u);

  EXPECT_EQ(track_222.getTrackLength(), 1u);
  EXPECT_EQ(track_222.getTrackId(), 2u);

  EXPECT_EQ(track_33.getTrackLength(), 2u);
  EXPECT_EQ(track_33.getTrackId(), 3u);

  // This checks to make sure the current frame is not part of any returned
  // track.
  for (auto it_track = tracks_at_time[6].begin();
       it_track != tracks_at_time[6].end(); ++it_track) {
    for (auto it_kid = it_track->getKeypointIdentifiers().begin();
         it_kid != it_track->getKeypointIdentifiers().end(); ++it_kid) {
      EXPECT_FALSE(
          it_kid->getFrameId() == nframes[6]->getFrameShared(0)->getId());
    }
  }
}

// Test whether the extractor can cut tracks that are above the limit.
TEST(FeatureTrackExtractor, TestTrackTruncating) {
  // Create 7 test frames so that we have the following tracks:
  //             F0    F1    F2    F3    F4    F5    F6    F7
  //  Track 0    *-----*--x--*-----*--x--*-----*--x--*
  //                            X (cut here, max. length=2)
  const size_t kCamera0Idx = 0u;
  aslam::NCamera::Ptr ncamera = aslam::NCamera::createTestNCamera(1);
  std::vector<aslam::VisualNFrame::Ptr> nframes;
  const size_t kNumFrame = 8;
  for (size_t c = 0; c < kNumFrame; ++c) {
    nframes.emplace_back(
        aslam::VisualNFrame::createEmptyTestVisualNFrame(ncamera, c));
  }

  // Create frame 0-6
  Eigen::VectorXi track_ids(1);
  Eigen::Matrix2Xd keypoints(2, 1);

  for (size_t frame_idx = 0; frame_idx < kNumFrame - 1; ++frame_idx) {
    track_ids << 1;
    keypoints.setOnes();
    nframes[frame_idx]->getFrameShared(0)->setTrackIds(track_ids);
    nframes[frame_idx]->getFrameShared(0)->setKeypointMeasurements(keypoints);
  }

  // One empty frames.
  track_ids << -1;
  nframes[kNumFrame - 1]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[kNumFrame - 1]->getFrameShared(0)->setKeypointMeasurements(keypoints);

  // Ground truth, there should be a track output at extraction step 2, 4 and 6
  // and 7.
  std::unordered_set<size_t> should_cut_at;
  should_cut_at.insert(2);
  should_cut_at.insert(4);
  should_cut_at.insert(6);

  const size_t kMaxTrackLength = 2;
  const size_t kMinTrackLength = 1;

  vio_common::FeatureTrackExtractor extractor(
      ncamera, kMaxTrackLength, kMinTrackLength);

  for (size_t i = 0; i < (kNumFrame - 1); ++i) {
    aslam::FeatureTracksList all_tracks;
    size_t num_tracks =
        extractor.extractFromNFrameStream(nframes[i], &all_tracks);
    ASSERT_EQ(all_tracks.size(), ncamera->getNumCameras());

    if (should_cut_at.count(i)) {
      EXPECT_EQ(1u, num_tracks);
      EXPECT_EQ(1u, all_tracks[kCamera0Idx].size());
      const aslam::FeatureTrack& track = all_tracks[kCamera0Idx].front();
      EXPECT_EQ(kMaxTrackLength, track.getTrackLength());
      EXPECT_EQ(track.getTrackId(), 1u);
      EXPECT_EQ(track.getTrackLength(), kMaxTrackLength);
    } else {
      EXPECT_EQ(0u, num_tracks);
      EXPECT_TRUE(all_tracks[kCamera0Idx].empty());
    }

    VLOG(3) << "Got " << num_tracks << " finished track at step " << i
            << std::endl;
  }
  // Add the last (empty) frame.
  aslam::FeatureTracksList all_tracks;
  size_t num_tracks =
      extractor.extractFromNFrameStream(nframes[kNumFrame - 1], &all_tracks);
  ASSERT_EQ(all_tracks.size(), ncamera->getNumCameras());
  // Expecting track of length 1 to terminate.
  EXPECT_EQ(num_tracks, 1u);
  ASSERT_EQ(all_tracks.size(), ncamera->getNumCameras());
  ASSERT_EQ(all_tracks[kCamera0Idx].size(), 1u);
  EXPECT_EQ(all_tracks[kCamera0Idx].front().getTrackLength(), 1u);
  EXPECT_EQ(all_tracks[kCamera0Idx].front().getTrackLength(), 1u);
  EXPECT_EQ(all_tracks[kCamera0Idx].front().getTrackId(), 1u);
}

// Test whether the extractor can handle empty frames. (channels are defined
// though!)
TEST(FeatureTrackExtractor, TestOutputIfNoKeypoints) {
  // Create empty frame
  Eigen::Matrix2Xd keypoints;
  keypoints.resize(Eigen::NoChange, 0);
  Eigen::VectorXi track_ids;
  track_ids.resize(0);

  const size_t kCamera0Idx = 0u;
  aslam::NCamera::Ptr ncamera = aslam::NCamera::createTestNCamera(1);
  aslam::VisualNFrame::Ptr nframe =
      aslam::VisualNFrame::createEmptyTestVisualNFrame(ncamera, 0);

  aslam::VisualFrame::Ptr frame = nframe->getFrameShared(0);
  frame->swapKeypointMeasurements(&keypoints);
  frame->swapTrackIds(&track_ids);

  const size_t kMaxTrackLength = 100;
  const size_t kMinTrackLength = 1;

  vio_common::FeatureTrackExtractor extractor(
      ncamera, kMaxTrackLength, kMinTrackLength);
  aslam::FeatureTracksList all_tracks;
  int num_tracks = extractor.extractFromNFrameStream(nframe, &all_tracks);

  EXPECT_EQ(num_tracks, 0);
  ASSERT_EQ(all_tracks.size(), ncamera->getNumCameras());
  EXPECT_TRUE(all_tracks[kCamera0Idx].empty());
}

TEST(FeatureTrackExtractor, TestExtractionWithTwoVisualFramesAndExtractBatch) {
  // Create test frames so that we have the following tracks:
  //                  F0     F1    F2    F3    F4
  // Cam0: Track 0    *------*-----*
  // Cam0: Track 1           *-----*-----*
  //
  // Cam1: Track 3    *------*
  // Cam1: Track 4                 *-----*

  aslam::NCamera::Ptr ncamera = aslam::NCamera::createTestNCamera(2);
  std::vector<aslam::VisualNFrame::Ptr> nframes;
  for (int c = 0; c < 5; ++c) {
    nframes.emplace_back(
        aslam::VisualNFrame::createEmptyTestVisualNFrame(ncamera, c));
  }

  Eigen::VectorXi track_ids;
  Eigen::Matrix2Xd keypoints;

  // Frame 0
  track_ids.resize(1);
  track_ids << 0;
  keypoints.resize(Eigen::NoChange, 1);
  nframes[0]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[0]->getFrameShared(0)->setKeypointMeasurements(keypoints);
  track_ids << 3;
  nframes[0]->getFrameShared(1)->setTrackIds(track_ids);
  nframes[0]->getFrameShared(1)->setKeypointMeasurements(keypoints);
  // Frame 1
  track_ids.resize(2);
  track_ids << 0, 1;
  keypoints.resize(Eigen::NoChange, 2);
  nframes[1]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[1]->getFrameShared(0)->setKeypointMeasurements(keypoints);
  track_ids.resize(1);
  track_ids << 3;
  keypoints.resize(Eigen::NoChange, 1);
  nframes[1]->getFrameShared(1)->setTrackIds(track_ids);
  nframes[1]->getFrameShared(1)->setKeypointMeasurements(keypoints);
  // Frame 2
  track_ids.resize(2);
  track_ids << 0, 1;
  keypoints.resize(Eigen::NoChange, 2);
  nframes[2]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[2]->getFrameShared(0)->setKeypointMeasurements(keypoints);
  track_ids.resize(1);
  track_ids << 4;
  keypoints.resize(Eigen::NoChange, 1);
  nframes[2]->getFrameShared(1)->setTrackIds(track_ids);
  nframes[2]->getFrameShared(1)->setKeypointMeasurements(keypoints);
  // Frame 3
  track_ids.resize(1);
  track_ids << 1;
  keypoints.resize(Eigen::NoChange, 1);
  nframes[3]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[3]->getFrameShared(0)->setKeypointMeasurements(keypoints);
  track_ids.resize(1);
  track_ids << 4;
  keypoints.resize(Eigen::NoChange, 1);
  nframes[3]->getFrameShared(1)->setTrackIds(track_ids);
  nframes[3]->getFrameShared(1)->setKeypointMeasurements(keypoints);
  // Frame 4
  track_ids.resize(0);
  keypoints.resize(Eigen::NoChange, 0);
  nframes[4]->getFrameShared(0)->setTrackIds(track_ids);
  nframes[4]->getFrameShared(0)->setKeypointMeasurements(keypoints);
  track_ids.resize(0);
  keypoints.resize(Eigen::NoChange, 0);
  nframes[4]->getFrameShared(1)->setTrackIds(track_ids);
  nframes[4]->getFrameShared(1)->setKeypointMeasurements(keypoints);

  // Ground truth data: which tracks should end at which frame
  // track_ids_groundtruth --> {
  //                             { {},{} },    // nframe0 (cam0, cam1)
  //                             { {},{} },    // nframe1 (cam0, cam1)
  //                             { {},{3} },   // nframe2 (cam0, cam1)
  //                             { {0},{} },   // nframe3 (cam0, cam1)
  //                             { {1},{4} },  // nframe4 (cam0, cam1)
  //                           };
  std::vector<std::vector<std::set<int>>> track_ids_groundtruth;
  track_ids_groundtruth.resize(5);
  // nframe0 (cam0, cam1)
  track_ids_groundtruth[0].emplace_back(std::set<int>());
  track_ids_groundtruth[0].emplace_back(std::set<int>());
  // nframe1 (cam0, cam1)
  track_ids_groundtruth[1].emplace_back(std::set<int>());
  track_ids_groundtruth[1].emplace_back(std::set<int>());
  // nframe2 (cam0, cam1)
  track_ids_groundtruth[2].emplace_back(std::set<int>());
  std::set<int> track1;
  track1.insert(3);
  track_ids_groundtruth[2].emplace_back(track1);
  // nframe3 (cam0, cam1)
  std::set<int> track2;
  track2.insert(0);
  track_ids_groundtruth[3].emplace_back(track2);
  track_ids_groundtruth[3].emplace_back(std::set<int>());
  // nframe4 (cam0, cam1)
  std::set<int> track3;
  track3.insert(1);
  std::set<int> track4;
  track4.insert(4);
  track_ids_groundtruth[4].emplace_back(track3);
  track_ids_groundtruth[4].emplace_back(track4);

  // Ground truth track lengths.
  std::vector<std::vector<size_t>> ground_truth_track_lengths(2);
  ground_truth_track_lengths[0].resize(2, 0);
  ground_truth_track_lengths[0][0] = 3;
  ground_truth_track_lengths[0][1] = 3;
  ground_truth_track_lengths[1].resize(5, 0);
  ground_truth_track_lengths[1][3] = 2;
  ground_truth_track_lengths[1][4] = 2;

  // Extract the tracks.
  const size_t kMaxTrackLength = 100;
  const size_t kMinTrackLength = 2;

  // Test stream extraction.
  vio_common::FeatureTrackExtractor extractor(
      ncamera, kMaxTrackLength, kMinTrackLength);
  for (int i = 0; i < 5; ++i) {
    aslam::FeatureTracksList rig_tracks;
    extractor.extractFromNFrameStream(nframes[i], &rig_tracks);
    size_t num_cameras = ncamera->getNumCameras();
    ASSERT_EQ(rig_tracks.size(), num_cameras);

    for (size_t cam_idx = 0; cam_idx < num_cameras; ++cam_idx) {
      for (const auto& track : rig_tracks[cam_idx]) {
        EXPECT_GT(
            track_ids_groundtruth[i][cam_idx].count(track.getTrackId()), 0u);
        EXPECT_GE(track.getTrackId(), 0u);
        EXPECT_EQ(
            ground_truth_track_lengths[cam_idx][track.getTrackId()],
            track.getTrackLength());
      }
    }
  }

  // Test the batch extraction.
  aslam::FeatureTracksList rig_tracks_batch;
  size_t num_cameras = ncamera->getNumCameras();
  size_t num_tracks = vio_common::FeatureTrackExtractor::extractBatch(
      nframes, &rig_tracks_batch);
  EXPECT_EQ(num_tracks, 4u);

  ASSERT_EQ(rig_tracks_batch.size(), num_cameras);
  ASSERT_EQ(rig_tracks_batch[0].size(), 2u);
  EXPECT_EQ(rig_tracks_batch[0][0].getTrackId(), 0u);
  EXPECT_EQ(rig_tracks_batch[0][0].getTrackLength(), 3u);
  EXPECT_EQ(rig_tracks_batch[0][1].getTrackId(), 1u);
  EXPECT_EQ(rig_tracks_batch[0][1].getTrackLength(), 3u);

  ASSERT_EQ(rig_tracks_batch[1].size(), 2u);
  EXPECT_EQ(rig_tracks_batch[1][0].getTrackId(), 3u);
  EXPECT_EQ(rig_tracks_batch[1][0].getTrackLength(), 2u);
  EXPECT_EQ(rig_tracks_batch[1][1].getTrackId(), 4u);
  EXPECT_EQ(rig_tracks_batch[1][1].getTrackLength(), 2u);
}

MAPLAB_UNITTEST_ENTRYPOINT
