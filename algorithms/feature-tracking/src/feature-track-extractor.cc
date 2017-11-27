#include <algorithm>
#include <iterator>

#include <unordered_map>
#include <unordered_set>

#include <Eigen/Dense>
#include <aslam/frames/keypoint-identifier.h>
#include <aslam/frames/visual-frame.h>
#include <aslam/frames/visual-nframe.h>
#include <glog/logging.h>

#include "feature-tracking/feature-track-extractor.h"

namespace vio_common {

FeatureTrackExtractor::FeatureTrackExtractor(
    const aslam::NCamera::ConstPtr& camera_rig)
    : FeatureTrackExtractor(
          camera_rig, std::numeric_limits<size_t>::max(), 2u) {}

FeatureTrackExtractor::FeatureTrackExtractor(
    const aslam::NCamera::ConstPtr& camera_rig, size_t max_track_length)
    : FeatureTrackExtractor(camera_rig, max_track_length, 2u) {}

FeatureTrackExtractor::FeatureTrackExtractor(
    const aslam::NCamera::ConstPtr& camera_rig, size_t max_track_length,
    size_t min_track_length)
    : camera_rig_(camera_rig),
      min_track_length_(min_track_length),
      max_track_length_(max_track_length) {
  CHECK_GT(min_track_length_, 0u);
  if (max_track_length_ > 0u) {
    CHECK_LE(min_track_length_, max_track_length_);
  }
  CHECK(camera_rig);
  const size_t num_cameras = camera_rig->getNumCameras();
  CHECK_GT(num_cameras, 0u);
  opportunistic_tracks_.resize(num_cameras);
  persistent_trackids_.resize(num_cameras);
}

size_t FeatureTrackExtractor::extractFromNFrameStream(
    const aslam::VisualNFrame::ConstPtr& nframe,
    aslam::FeatureTracksList* tracks_opportunistic_terminated) {
  CHECK(nframe);
  CHECK_NOTNULL(tracks_opportunistic_terminated)->clear();
  const size_t num_cameras = nframe->getNumFrames();
  CHECK_GT(num_cameras, 0u);
  tracks_opportunistic_terminated->resize(num_cameras);
  aslam::FeatureTracksList tracks_persistent_new(num_cameras);
  std::vector<aslam::ContinuedFeatureTracks> tracks_persistent_continued(
      num_cameras);
  std::vector<std::unordered_set<int>> tracks_persistent_terminated(
      num_cameras);

  // TODO(schneith): Use the thread pool at some point.
  const bool kTrackPersistentFeatures = false;
  size_t num_tracks = 0;
  for (size_t camera_idx = 0u; camera_idx < num_cameras; ++camera_idx) {
    (*tracks_opportunistic_terminated)[camera_idx].reserve(
        nframe->getFrame(camera_idx).getNumKeypointMeasurements());
    extractFromFrameStreamImpl(
        nframe, camera_idx, kTrackPersistentFeatures,
        &(*tracks_opportunistic_terminated)[camera_idx],
        &tracks_persistent_new[camera_idx],
        &tracks_persistent_continued[camera_idx],
        &tracks_persistent_terminated[camera_idx]);

    // This method doesn't track persistent features, therefore the following
    // lists have to be
    // empty.
    CHECK(tracks_persistent_new[camera_idx].empty());
    CHECK(tracks_persistent_continued[camera_idx].empty());
    CHECK(tracks_persistent_terminated[camera_idx].empty());

    num_tracks += (*tracks_opportunistic_terminated)[camera_idx].size();
  }
  previous_nframe_ = nframe;
  return num_tracks;
}

void FeatureTrackExtractor::extractFromNFrameStream(
    const aslam::VisualNFrame::ConstPtr& nframe,
    aslam::FeatureTracksList* tracks_opportunistic_terminated,
    aslam::FeatureTracksList* tracks_persistent_new,
    std::vector<aslam::ContinuedFeatureTracks>* tracks_persistent_continued,
    std::vector<std::unordered_set<int>>* tracks_persistent_terminated) {
  CHECK(nframe);
  CHECK_NOTNULL(tracks_opportunistic_terminated)->clear();
  CHECK_NOTNULL(tracks_persistent_new)->clear();
  CHECK_NOTNULL(tracks_persistent_continued)->clear();
  CHECK_NOTNULL(tracks_persistent_terminated)->clear();

  const size_t num_cameras = nframe->getNumFrames();
  CHECK_GT(num_cameras, 0u);
  tracks_opportunistic_terminated->resize(num_cameras);
  tracks_persistent_new->resize(num_cameras);
  tracks_persistent_continued->resize(num_cameras);
  tracks_persistent_terminated->resize(num_cameras);

  // TODO(schneith): Use the thread pool at some point.
  const bool kTrackpersistentFeatures = true;
  const double kKeypointToTrackRatioGuess = 0.5;
  const size_t num_reserve = kKeypointToTrackRatioGuess *
                             nframe->getFrame(0).getNumKeypointMeasurements();
  for (size_t camera_idx = 0u; camera_idx < num_cameras; ++camera_idx) {
    (*tracks_opportunistic_terminated)[camera_idx].reserve(num_reserve);
    (*tracks_opportunistic_terminated)[camera_idx].reserve(num_reserve);
    (*tracks_persistent_continued)[camera_idx].reserve(num_reserve);
    (*tracks_persistent_terminated)[camera_idx].reserve(num_reserve);
    extractFromFrameStreamImpl(
        nframe, camera_idx, kTrackpersistentFeatures,
        &(*tracks_opportunistic_terminated)[camera_idx],
        &(*tracks_persistent_new)[camera_idx],
        &(*tracks_persistent_continued)[camera_idx],
        &(*tracks_persistent_terminated)[camera_idx]);
  }
  previous_nframe_ = nframe;
}

void FeatureTrackExtractor::extractFromNFrameStream(
    const aslam::VisualNFrame::ConstPtr& nframe,
    aslam::FeatureTracks* tracks_opportunistic_terminated,
    aslam::FeatureTracks* tracks_persistent_new,
    aslam::ContinuedFeatureTracks* tracks_persistent_continued,
    std::unordered_set<int>* tracks_persistent_terminated) {
  CHECK(nframe);
  CHECK_NOTNULL(tracks_opportunistic_terminated)->clear();
  CHECK_NOTNULL(tracks_persistent_new)->clear();
  CHECK_NOTNULL(tracks_persistent_continued)->clear();
  CHECK_NOTNULL(tracks_persistent_terminated)->clear();

  // Extract for each camera individually.
  const size_t num_cameras = nframe->getNumFrames();
  CHECK_GT(num_cameras, 0u);
  aslam::FeatureTracksList rig_tracks_opportunistic_terminated;
  aslam::FeatureTracksList rig_tracks_persistent_new;
  std::vector<aslam::ContinuedFeatureTracks> rig_tracks_persistent_continued;
  std::vector<std::unordered_set<int>> rig_tracks_persistent_terminated;
  extractFromNFrameStream(
      nframe, &rig_tracks_opportunistic_terminated, &rig_tracks_persistent_new,
      &rig_tracks_persistent_continued, &rig_tracks_persistent_terminated);
  CHECK_EQ(num_cameras, rig_tracks_opportunistic_terminated.size());
  CHECK_EQ(num_cameras, rig_tracks_persistent_new.size());
  CHECK_EQ(num_cameras, rig_tracks_persistent_continued.size());
  CHECK_EQ(num_cameras, rig_tracks_persistent_terminated.size());

  // Join all camera tracks into container.
  for (size_t camera_idx = 0u; camera_idx < num_cameras; ++camera_idx) {
    tracks_opportunistic_terminated->insert(
        tracks_opportunistic_terminated->end(),
        rig_tracks_opportunistic_terminated[camera_idx].begin(),
        rig_tracks_opportunistic_terminated[camera_idx].end());
    tracks_persistent_new->insert(
        tracks_persistent_new->end(),
        rig_tracks_persistent_new[camera_idx].begin(),
        rig_tracks_persistent_new[camera_idx].end());
    tracks_persistent_continued->insert(
        tracks_persistent_continued->end(),
        rig_tracks_persistent_continued[camera_idx].begin(),
        rig_tracks_persistent_continued[camera_idx].end());
    tracks_persistent_terminated->insert(
        rig_tracks_persistent_terminated[camera_idx].begin(),
        rig_tracks_persistent_terminated[camera_idx].end());
  }
  previous_nframe_ = nframe;
}

void FeatureTrackExtractor::extractFromFrameStreamImpl(
    const aslam::VisualNFrame::ConstPtr& nframe, size_t camera_idx,
    bool track_persistent_features,
    aslam::FeatureTracks* tracks_opportunistic_terminated,
    aslam::FeatureTracks* tracks_persistent_new,
    aslam::ContinuedFeatureTracks* tracks_persistent_continued,
    std::unordered_set<int>* tracks_persistent_terminated) {
  CHECK(nframe);
  CHECK_EQ(camera_rig_.get(), nframe->getNCameraShared().get());
  CHECK_LT(camera_idx, nframe->getNumCameras());
  CHECK_LT(camera_idx, opportunistic_tracks_.size());
  CHECK_LT(camera_idx, persistent_trackids_.size());
  const aslam::VisualFrame& frame = nframe->getFrame(camera_idx);
  CHECK(frame.hasTrackIds());
  CHECK(frame.hasKeypointMeasurements());
  CHECK_NOTNULL(tracks_opportunistic_terminated);
  CHECK_NOTNULL(tracks_persistent_new);
  CHECK_NOTNULL(tracks_persistent_continued);
  CHECK_NOTNULL(tracks_persistent_terminated);

  // If there is a previous frame, we need to build a LUT mapping track id to
  // keypoint index in
  // order to be able to quickly look up the keypoint index in the previous
  // frame for a given new
  // track id in the current frame.
  typedef std::unordered_map<int, int> TrackIdToKeypointIndexMap;
  TrackIdToKeypointIndexMap previous_frame_track_id_keypoint_index_map;
  if (previous_nframe_) {
    const Eigen::VectorXi& previous_track_ids =
        previous_nframe_->getFrame(camera_idx).getTrackIds();

    const size_t num_track_ids = static_cast<size_t>(previous_track_ids.rows());
    previous_frame_track_id_keypoint_index_map.reserve(num_track_ids);

    for (size_t keypoint_idx = 0u; keypoint_idx < num_track_ids;
         ++keypoint_idx) {
      const int track_id = previous_track_ids(keypoint_idx);
      if (track_id >= 0) {
        previous_frame_track_id_keypoint_index_map.insert(
            std::make_pair(track_id, keypoint_idx));
      }
    }
  }

  // Find currently new and continued tracks.
  std::unordered_set<int> present_tracks;
  const Eigen::VectorXi& current_track_ids = frame.getTrackIds();
  const size_t num_keypoints = frame.getNumKeypointMeasurements();
  size_t num_track_ids = static_cast<size_t>(current_track_ids.rows());
  CHECK_EQ(num_keypoints, num_track_ids);

  // Go through all tracks ids and check if the track is new, continued,
  // terminated or persistent.
  for (size_t keypoint_idx_current_frame = 0u;
       keypoint_idx_current_frame < num_track_ids;
       ++keypoint_idx_current_frame) {
    const int track_id = current_track_ids(keypoint_idx_current_frame);

    if (track_id >= 0) {
      CHECK_LT(keypoint_idx_current_frame, num_keypoints);
      // Add all valid tracks_ids seen in this frame to a list that is compared
      // to the list of
      // opportunistic tracks to determine which tracks have terminated.
      present_tracks.insert(track_id);

      TrackIdToFeatureTrackMap::iterator it_track =
          opportunistic_tracks_[camera_idx].find(track_id);
      if (it_track != opportunistic_tracks_[camera_idx].end()) {
        // This is a continued track as there is already an entry in the books.
        aslam::FeatureTrack& continued_track = it_track->second;
        const int continued_track_id = continued_track.getTrackId();
        CHECK_EQ(track_id, continued_track_id);

        // If the track length already reached max-length but the tracks hasn't
        // terminated yet,
        // we either cut the track and start a new one or convert it into a
        // persistent track
        // depending on the settings of track_persistent_features.
        if (continued_track.getTrackLength() >= max_track_length_) {
          if (track_persistent_features) {
            // Return the current track and convert it to a persistent track.
            tracks_persistent_new->emplace_back(continued_track);
            opportunistic_tracks_[camera_idx].erase(it_track);
            // Add it to the list of persistent tracks. This will avoid that a
            // new opportunistic
            // track is spawned for this trackid in the next update and future
            // observations are
            // returned as ContinuedFeatureTracks messages.
            persistent_trackids_[camera_idx].insert(continued_track_id);
            VLOG(100) << "New persistent track with track id " << track_id
                      << " in camera " << camera_idx;
          } else {
            // Add the track to the terminated tracks. It ends at the previous
            // frame.
            tracks_opportunistic_terminated->emplace_back(continued_track);
            opportunistic_tracks_[camera_idx].erase(it_track);

            // Cut and start a new track starting at the current frame.
            aslam::FeatureTrack new_track(track_id);
            new_track.addKeypointObservationAtBack(
                nframe, camera_idx, keypoint_idx_current_frame);
            CHECK(
                opportunistic_tracks_[camera_idx]
                    .emplace(track_id, new_track)
                    .second);
          }
        } else {
          // Simply append the current keypoint to the existing track.
          continued_track.addKeypointObservationAtBack(
              nframe, camera_idx, keypoint_idx_current_frame);
        }
      } else {
        // This is a new track as the TrackId isn't in the books so far. Either
        // we need to start
        // a new track or we output a track continuation message.
        std::unordered_set<int>::const_iterator it =
            persistent_trackids_[camera_idx].find(track_id);
        const bool is_persistent_track =
            (it != persistent_trackids_[camera_idx].end());

        if (is_persistent_track) {
          // Output the ContinuedFeatureTracks message.
          CHECK(previous_nframe_);
          TrackIdToKeypointIndexMap::iterator it =
              previous_frame_track_id_keypoint_index_map.find(track_id);
          CHECK(it != previous_frame_track_id_keypoint_index_map.end());
          const int keypoint_idx_in_previous_frame = it->second;
          tracks_persistent_continued->emplace_back(
              track_id, aslam::KeypointIdentifier::create(
                            previous_nframe_, camera_idx,
                            keypoint_idx_in_previous_frame));
        } else {
          // This is a new track.
          aslam::FeatureTrack new_track(track_id);

          // If there is a previous frame and if we can find the track id in the
          // previous frame,
          // add its keypoint identifier. This allows but does not force full
          // track initialization.
          // This is needed as the tracker writes matches to the frame k-1 and k
          // when starting a
          // new track.
          if (previous_nframe_) {
            TrackIdToKeypointIndexMap::iterator it =
                previous_frame_track_id_keypoint_index_map.find(track_id);
            if (it != previous_frame_track_id_keypoint_index_map.end()) {
              // Found the track id in the previous frame.
              const int keypoint_idx_in_previous_frame = it->second;
              new_track.addKeypointObservationAtBack(
                  previous_nframe_, camera_idx, keypoint_idx_in_previous_frame);
            }
          }

          new_track.addKeypointObservationAtBack(
              nframe, camera_idx, keypoint_idx_current_frame);
          opportunistic_tracks_[camera_idx].emplace(track_id, new_track);
        }
      }
    }
  }

  // Check for opportunistic tracks which terminated in the last frame.
  TrackIdToFeatureTrackMap::iterator track_it =
      opportunistic_tracks_[camera_idx].begin();
  while (track_it != opportunistic_tracks_[camera_idx].end()) {
    // If it is not new or continued, it is a terminated track!
    const int feature_track_id = track_it->first;

    if (present_tracks.count(feature_track_id) == 0u) {
      // Track is not new or continued, so it is a terminated track.
      const aslam::FeatureTrack& finished_track = track_it->second;
      // Only return tracks whose length is at least the min. track length.
      if (finished_track.getTrackLength() >= min_track_length_) {
        tracks_opportunistic_terminated->push_back(finished_track);
        VLOG(200) << "Track finished with id " << feature_track_id
                  << " and length " << finished_track.getTrackLength();
      }
      track_it = opportunistic_tracks_[camera_idx].erase(track_it);
    } else {
      ++track_it;
    }
  }

  // Check for persistent tracks that terminated in the last frame.
  std::unordered_set<int>::iterator trackid_it =
      persistent_trackids_[camera_idx].begin();
  while (trackid_it != persistent_trackids_[camera_idx].end()) {
    // If it is not new or continued, it is a terminated track!
    const int feature_track_id = *trackid_it;

    // If persistent trackid was not seen during this update, we terminate the
    // persistent track.
    if (present_tracks.count(feature_track_id) == 0u) {
      tracks_persistent_terminated->insert(feature_track_id);
      trackid_it = persistent_trackids_[camera_idx].erase(trackid_it);
    } else {
      ++trackid_it;
    }
  }
}

size_t FeatureTrackExtractor::extractBatch(
    const aslam::VisualNFrame::ConstPtrVector& nframes,
    aslam::FeatureTracksList* all_tracks) {
  CHECK_NOTNULL(all_tracks);
  CHECK(!nframes.empty());
  all_tracks->clear();

  // Setup the extractor.
  const aslam::NCamera::ConstPtr& ncamera =
      CHECK_NOTNULL(nframes.front().get())->getNCameraShared();
  CHECK(ncamera);
  size_t num_cameras = ncamera->getNumCameras();
  all_tracks->resize(num_cameras);

  const size_t num_nframes = nframes.size();
  const size_t max_track_length = num_nframes;
  FeatureTrackExtractor extractor(ncamera, max_track_length);

  auto appendToOutput = [&num_cameras](
      const aslam::FeatureTracksList& rig_tracks,
      aslam::FeatureTracksList* output_rig_tracks) {
    CHECK_NOTNULL(output_rig_tracks);
    CHECK_EQ(output_rig_tracks->size(), rig_tracks.size());

    for (size_t cam_idx = 0; cam_idx < num_cameras; ++cam_idx) {
      (*output_rig_tracks)[cam_idx].insert(
          (*output_rig_tracks)[cam_idx].end(), rig_tracks[cam_idx].begin(),
          rig_tracks[cam_idx].end());
    }
  };

  // Extract all tracks and append to output.
  size_t num_total_tracks = 0;
  aslam::FeatureTracksList all_terminated_tracks;
  for (const aslam::VisualNFrame::ConstPtr& nframe : nframes) {
    num_total_tracks +=
        extractor.extractFromNFrameStream(nframe, &all_terminated_tracks);
    appendToOutput(all_terminated_tracks, all_tracks);
  }

  // Abort the rest of the tracks and append to output.
  aslam::FeatureTracksList all_aborted_tracks;
  num_total_tracks +=
      extractor.abortAndReturnOpportunisticTracks(&all_aborted_tracks);
  appendToOutput(all_aborted_tracks, all_tracks);

  return num_total_tracks;
}

size_t FeatureTrackExtractor::abortAndReturnOpportunisticTracks(
    aslam::FeatureTracksList* all_tracks) {
  CHECK_NOTNULL(all_tracks);

  const size_t num_cameras = camera_rig_->getNumCameras();
  all_tracks->clear();
  all_tracks->resize(num_cameras);

  // Get all opportunistic tracks.
  size_t num_tracks = 0;
  for (size_t camera_idx = 0u; camera_idx < num_cameras; ++camera_idx) {
    for (const TrackIdToFeatureTrackMap::value_type&
             active_opportunistic_track : opportunistic_tracks_[camera_idx]) {
      if (active_opportunistic_track.second.getTrackLength() >=
          min_track_length_) {
        (*all_tracks)[camera_idx].push_back(active_opportunistic_track.second);
        ++num_tracks;
      }
    }
  }

  // Reset all tracks.
  reset();
  return num_tracks;
}

void FeatureTrackExtractor::abortAndReturnNLongestOpportunisticTracks(
    size_t num_tracks_to_abort, size_t min_track_length,
    aslam::FeatureTracks* aborted_tracks) {
  CHECK_NOTNULL(aborted_tracks)->clear();
  CHECK_GT(min_track_length, 0u);
  const size_t num_cameras = camera_rig_->getNumCameras();
  aborted_tracks->reserve(num_tracks_to_abort);

  // Build an index of track length over all cameras.
  typedef std::pair<size_t, int> CameraIdxTrackIdPair;
  typedef std::map<size_t, CameraIdxTrackIdPair>
      TrackLengthOpportunisticTrackMap;
  TrackLengthOpportunisticTrackMap tracklength_track_map;
  for (size_t camera_idx = 0u; camera_idx < num_cameras; ++camera_idx) {
    for (const TrackIdToFeatureTrackMap::value_type&
             active_opportunistic_track : opportunistic_tracks_[camera_idx]) {
      const aslam::FeatureTrack& track = active_opportunistic_track.second;
      if (track.getTrackLength() >= min_track_length) {
        tracklength_track_map.emplace(
            std::make_pair(
                track.getTrackLength(),
                CameraIdxTrackIdPair(camera_idx, track.getTrackId())));
      }
    }
  }

  // Abort and return the N longest tracks.
  TrackLengthOpportunisticTrackMap::reverse_iterator it_length_trackid =
      tracklength_track_map.rbegin();
  for (; it_length_trackid != tracklength_track_map.rend();
       ++it_length_trackid) {
    const size_t camera_idx = it_length_trackid->second.first;
    const int track_id = it_length_trackid->second.second;
    CHECK_GE(track_id, 0);

    // Find the active opportunistic track.
    CHECK_LT(camera_idx, opportunistic_tracks_.size());
    TrackIdToFeatureTrackMap::iterator it_active_opportunistic_track =
        opportunistic_tracks_[camera_idx].find(track_id);
    CHECK(
        it_active_opportunistic_track !=
        opportunistic_tracks_[camera_idx].end());

    // Add to output and abort the active track.
    aborted_tracks->emplace_back(it_active_opportunistic_track->second);
    opportunistic_tracks_[camera_idx].erase(it_active_opportunistic_track);

    if (aborted_tracks->size() >= num_tracks_to_abort) {
      break;
    }
  }
  CHECK_LE(aborted_tracks->size(), num_tracks_to_abort);
}

void FeatureTrackExtractor::abortPersistentTracksByTrackId(
    const std::unordered_set<int>& track_ids_to_abort) {
  // We need to check all cameras as we don't know in which camera the track
  // originated.
  for (size_t camera_idx = 0u; camera_idx < persistent_trackids_.size();
       ++camera_idx) {
    for (int track_id : track_ids_to_abort) {
      CHECK_GE(track_id, 0);
      persistent_trackids_[camera_idx].erase(track_id);
    }
  }
}

size_t FeatureTrackExtractor::getNumOpportunisticTracks(
    size_t camera_idx) const {
  CHECK_LT(camera_idx, opportunistic_tracks_.size());
  return opportunistic_tracks_[camera_idx].size();
}

size_t FeatureTrackExtractor::getNumPersistentTracks(size_t camera_idx) const {
  CHECK_LT(camera_idx, persistent_trackids_.size());
  return persistent_trackids_[camera_idx].size();
}

void FeatureTrackExtractor::reset() {
  const size_t num_cameras = camera_rig_->getNumCameras();
  CHECK_EQ(num_cameras, opportunistic_tracks_.size());
  CHECK_EQ(num_cameras, persistent_trackids_.size());
  for (size_t camera_idx = 0u; camera_idx < num_cameras; ++camera_idx) {
    opportunistic_tracks_[camera_idx].clear();
    persistent_trackids_[camera_idx].clear();
  }
}

}  // namespace vio_common
