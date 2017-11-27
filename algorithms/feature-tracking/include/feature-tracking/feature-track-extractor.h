#ifndef FEATURE_TRACKING_FEATURE_TRACK_EXTRACTOR_H_
#define FEATURE_TRACKING_FEATURE_TRACK_EXTRACTOR_H_

#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <aslam/common/memory.h>
#include <aslam/frames/feature-track.h>
#include <maplab-common/macros.h>

namespace aslam {
class NCamera;
class VisualFrame;
class VisualNFrame;
}

namespace vio_common {

/// \class FeatureTrackExtractor
/// \brief This class can be used to extract feature tracks from a stream of
/// VisualFrames that
///        contain the TrackIDs channel.
class FeatureTrackExtractor {
 public:
  MAPLAB_POINTER_TYPEDEFS(FeatureTrackExtractor);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //////////////////////////////////////////////////////////////
  /// \name Constructors and operators.
  /// @
 public:
  FeatureTrackExtractor() = delete;
  /// Default constructor with a default value for min_track_length of 2 and max
  /// track length of 1000. Note that a realistic max track length is
  /// necessary because memory is pre-allocated according to that length.
  explicit FeatureTrackExtractor(
      const std::shared_ptr<const aslam::NCamera>& camera_rig);

  /// Constructor with a default value for min_track_length of 2.
  FeatureTrackExtractor(
      const std::shared_ptr<const aslam::NCamera>& camera_rig,
      size_t max_track_length);
  FeatureTrackExtractor(
      const std::shared_ptr<const aslam::NCamera>& camera_rig,
      size_t max_track_length, size_t min_track_length);

  ~FeatureTrackExtractor() {}
  /// @}

  //////////////////////////////////////////////////////////////
  /// \name Track extraction methods.
  /// @{
 public:
  /// \brief Extracts feature tracks from a VisualNFrame stream for all cameras.
  /// @param[in] nframe           Current VisualNFrame to extract the tracks
  /// from.
  /// @param[out] all_tracks      Vector of tracks that ended in the previous
  /// frame. Tracks
  ///             reaching max track length are cut and also returned such that
  ///             their last
  ///             referenced frame is the previous frame. The outer vector
  ///             corresponds to the
  ///             cameras in the rig.
  /// @return Number of finished tracks over all cameras.
  size_t extractFromNFrameStream(
      const std::shared_ptr<const aslam::VisualNFrame>& nframe,
      aslam::FeatureTracksList* all_tracks);

  /// \brief Extracts feature tracks from a VisualNFrame stream for all cameras.
  ///        This method overload returns the tracks separated for each camera.
  /// @param[in]  nframe   Current nframe containing the tracks.
  /// @param[out] tracks_opportunistic_terminated   Terminated opportunistic
  /// feature tracks.
  ///             These tracks terminated before reaching the max. length and
  ///             are no longer
  ///             tracked.
  /// @param[out] tracks_persistent_new   New persistent feature tracks. These
  /// tracks reached the
  ///             max. length and are returned for triangulation and processing.
  ///             Further
  ///             observations of this track are returned as
  ///             tracks_persistent_continued messages.
  /// @param[out] tracks_persistent_continued    Continued persistent feature
  /// tracks.
  /// @param[out] tracks_persistent_terminated   Terminated persistent feature
  /// tracks.
  void extractFromNFrameStream(
      const aslam::VisualNFrame::ConstPtr& nframe,
      aslam::FeatureTracksList* tracks_opportunistic_terminated,
      aslam::FeatureTracksList* tracks_persistent_new,
      std::vector<aslam::ContinuedFeatureTracks>* tracks_persistent_continued,
      std::vector<std::unordered_set<int>>* tracks_persistent_terminated);

  /// Same as extractFromFrameStream but the tracks of all cameras are joined
  /// into one list.
  void extractFromNFrameStream(
      const aslam::VisualNFrame::ConstPtr& nframe,
      aslam::FeatureTracks* tracks_opportunistic_terminated,
      aslam::FeatureTracks* tracks_persistent_new,
      aslam::ContinuedFeatureTracks* tracks_persistent_continued,
      std::unordered_set<int>* tracks_persistent_terminated);

  /// \brief Extracts all feature tracks given a list of nframes.
  /// @param[in]  nframes         List of VisualNFrame to extract the tracks
  /// from.
  /// @param[out] all_tracks      All feature tracks.
  /// @return Number of finished tracks over all cameras.
  static size_t extractBatch(
      const std::vector<std::shared_ptr<const aslam::VisualNFrame>>& nframes,
      aslam::FeatureTracksList* all_tracks);

  /// Forwarder for NonConstPtr vectors.
  static inline size_t extractBatch(
      const std::vector<std::shared_ptr<aslam::VisualNFrame>>& nframes,
      aslam::FeatureTracksList* all_tracks) {
    std::vector<std::shared_ptr<const aslam::VisualNFrame>> nframes_cast(
        nframes.begin(), nframes.end());
    return extractBatch(nframes_cast, all_tracks);
  }

  /// Abort and return all opportunistic tracks. Returns the total number of
  /// tracks over all
  /// cameras.
  size_t abortAndReturnOpportunisticTracks(aslam::FeatureTracksList* tracks);

  /// Abort and return the N longest opportunistic tracks.
  void abortAndReturnNLongestOpportunisticTracks(
      size_t num_tracks_to_abort, size_t min_track_length,
      aslam::FeatureTracks* aborted_tracks);

  /// Abort a list of persistent features specified by the TrackId. All cameras
  /// are searched for
  /// TrackId matches.
  void abortPersistentTracksByTrackId(
      const std::unordered_set<int>& track_ids_to_abort);

  /// Get the current number of opportunistic feature tracks.
  size_t getNumOpportunisticTracks(size_t camera_idx) const;

  /// Get the current number of persistent feature tracks.
  size_t getNumPersistentTracks(size_t camera_idx) const;

  /// Set the max. track length. Already active opportunistic tracks over this
  /// length will be
  /// returned in the next call to extractFromNFrameStream.
  void setMaxTrackLength(size_t max_track_length) {
    CHECK_GT(max_track_length, 0u);
    max_track_length_ = max_track_length;
  }

  /// Reset all internal states.
  void reset();
  /// @}

  //////////////////////////////////////////////////////////////
  /// \name Internal implementations.
  /// @{
 private:
  /// Single camera implementation for the stream extractor.
  void extractFromFrameStreamImpl(
      const aslam::VisualNFrame::ConstPtr& nframe, size_t camera_idx,
      bool track_persistent_features,
      aslam::FeatureTracks* tracks_opportunistic_terminated,
      aslam::FeatureTracks* tracks_persistent_new,
      aslam::ContinuedFeatureTracks* tracks_persistent_contiued,
      std::unordered_set<int>* tracks_persistent_terminated);
  /// @}

 private:
  const std::shared_ptr<const aslam::NCamera> camera_rig_;

  typedef std::unordered_map<int, aslam::FeatureTrack> TrackIdToFeatureTrackMap;
  /// Currently opportunistic tracks stored with their id.
  std::vector<TrackIdToFeatureTrackMap> opportunistic_tracks_;
  /// Currently persistent tracks stored with their id.
  std::vector<std::unordered_set<int>> persistent_trackids_;
  /// Pointer to the previous frame.
  std::shared_ptr<const aslam::VisualNFrame> previous_nframe_;

  /// The minimum track length. Shorter tracks get discarded and are not
  /// returned.
  const size_t min_track_length_;
  /// The maximum track length. All returned tracks have length <=
  /// max_track_length. Tracks
  /// get cut appropriately.
  /// Track length 0 is used to encode infinite track length.
  size_t max_track_length_;
};

}  // namespace vio_common
#endif  // FEATURE_TRACKING_FEATURE_TRACK_EXTRACTOR_H_
