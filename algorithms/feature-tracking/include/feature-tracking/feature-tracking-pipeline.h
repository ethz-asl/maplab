#ifndef FEATURE_TRACKING_FEATURE_TRACKING_PIPELINE_H_
#define FEATURE_TRACKING_FEATURE_TRACKING_PIPELINE_H_

#include <string>
#include <vector>

#include <aslam/common/memory.h>
#include <aslam/common/statistics/accumulator.h>
#include <aslam/frames/feature-track.h>
#include <aslam/visualization/feature-track-visualizer.h>
#include <maplab-common/macros.h>
#include <posegraph/unique-id.h>

#include "feature-tracking/feature-detection-extraction.h"
#include "feature-tracking/feature-track-extractor.h"

namespace aslam {
class FeatureTrackerLk;
class VisualNFrame;
}

namespace vi_map {
class VIMap;
class MissionId;
}

namespace aslam_cv_visualization {
class VisualNFrameFeatureTrackVisualizer;
}

namespace feature_tracking {

/// Pipeline to rerun tracking of features and triangulation of landmarks on a
/// already existing VIMap (i.e. with pose-graph, etc.).
/// Visualization of keypoints, keypoint matches and feature tracks is
/// available. See the flags at the top of the the source file.
class FeatureTrackingPipeline {
 public:
  MAPLAB_POINTER_TYPEDEFS(FeatureTrackingPipeline);
  MAPLAB_DISALLOW_EVIL_CONSTRUCTORS(FeatureTrackingPipeline);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FeatureTrackingPipeline();
  virtual ~FeatureTrackingPipeline() = default;

  /// Reruns tracking and triangulation on all missions of the given map.
  /// Note: This at first removes all existing landmarks and observations in the
  /// map!
  void runTrackingAndTriangulationForAllMissions(vi_map::VIMap* map);

  /// Rerun tracking and triangulation for a given mission.
  /// Note: This at first removes all existing landmarks and observations in the
  /// map!
  void runTrackingAndTriangulationForMission(
      vi_map::MissionId mission_id, vi_map::VIMap* map);

 protected:
  inline bool hasFirstNFrameBeenProcessed() const {
    return processed_first_nframe_;
  }

  const std::string feature_tracking_ros_base_topic_;
  const bool visualize_keypoint_matches_;

 private:
  virtual void initialize(const aslam::NCamera::ConstPtr& ncamera) = 0;
  virtual void trackFeaturesNFrame(
      const aslam::Transformation& T_Bk_Bkp1, aslam::VisualNFrame* nframe_k,
      aslam::VisualNFrame* nframe_kp1) = 0;

  // Loads the raw-images specified in the resources table of the given map and
  // assigns them to the frames of the nframe of the given vertex.
  void assignRawImagesToNFrame(
      const pose_graph::VertexId& vertex_id, vi_map::VIMap* map) const;

  // Looks for terminated feature tracks, triangulates them and adds them to the
  // given map.
  void extractAndTriangulateTerminatedFeatureTracks(
      const aslam::VisualNFrame::ConstPtr& nframe, vi_map::VIMap* map);

  aslam_cv_visualization::VisualNFrameFeatureTrackVisualizer
      feature_track_visualizer_;

  vio_common::FeatureTrackExtractor::UniquePtr track_extractor_;

  typedef AlignedUnorderedMap<aslam::NFramesId, pose_graph::VertexId>
      NFrameIdToVertexIdMap;
  NFrameIdToVertexIdMap nframe_id_to_vertex_id_map_;

  bool processed_first_nframe_;

  statistics::Accumulator<size_t, size_t, statistics::kInfiniteWindowSize>
      successfully_triangulated_landmarks_accumulator_;
};
}  // namespace feature_tracking

#endif  // FEATURE_TRACKING_FEATURE_TRACKING_PIPELINE_H_
