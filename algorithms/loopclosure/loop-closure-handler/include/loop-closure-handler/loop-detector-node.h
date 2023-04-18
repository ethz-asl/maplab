#ifndef LOOP_CLOSURE_HANDLER_LOOP_DETECTOR_NODE_H_
#define LOOP_CLOSURE_HANDLER_LOOP_DETECTOR_NODE_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <aslam/common/memory.h>
#include <descriptor-projection/descriptor-projection.h>
#include <localization-summary-map/unique-id.h>
#include <loopclosure-common/types.h>
#include <maplab-common/file-serializable.h>
#include <sensors/external-features.h>
#include <vi-map/mission-baseframe.h>
#include <vi-map/unique-id.h>
#include <vi-map/vi-map.h>

#include "loop-closure-handler/loop-closure-constraint.h"
#include "loop-closure-handler/loop-closure-handler.h"
#include "loop-closure-handler/visualization/loop-closure-visualizer.h"

namespace matching_based_loopclosure {
class LoopDetector;
}  // namespace matching_based_loopclosure

namespace loop_closure {
struct ProjectedImage;
}  // namespace loop_closure

namespace loop_detector_node {

// Ability to mix data from multiple maps intended.
class LoopDetectorNode final {
 public:
  MAPLAB_POINTER_TYPEDEFS(LoopDetectorNode);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef vi_map::MissionBaseFrameMap MissionBaseFrameMap;
  typedef vi_map::MissionMap MissionMap;
  typedef vi_map::MissionId MissionId;
  typedef std::unordered_map<vi_map::LandmarkId, vi_map::LandmarkId>
      LandmarkToLandmarkMap;

  LoopDetectorNode();

  void detectLoopClosuresAndMergeLandmarks(
      const MissionId& mission, vi_map::VIMap* map);

  void addVertexToDatabase(
      const pose_graph::VertexId& vertex_id, const vi_map::VIMap& map);

  void addMissionToDatabase(
      const vi_map::MissionId& mission_id, const vi_map::VIMap& map);
  void addVerticesToDatabase(
      const pose_graph::VertexIdList& vertex_ids, const vi_map::VIMap& map);

  bool hasMissionInDatabase(const vi_map::MissionId& mission_id) const;

  void addLocalizationSummaryMapToDatabase(
      const summary_map::LocalizationSummaryMap& localization_summary_map);

  bool findNFrameInSummaryMapDatabase(
      const aslam::VisualNFrame& n_frame, const bool skip_untracked_keypoints,
      const summary_map::LocalizationSummaryMap& localization_summary_map,
      pose::Transformation* T_G_I, unsigned int* num_of_lc_matches,
      vi_map::VertexKeyPointToStructureMatchList* inlier_structure_matches)
      const;

  bool detectLoopClosuresMissionToDatabase(
      const MissionId& mission_id, const bool merge_landmarks,
      const bool add_lc_edges, vi_map::VIMap* map,
      pose::Transformation* T_G_M_estimate,
      vi_map::LoopClosureConstraintVector* inlier_constraints) const;

  bool detectLoopClosuresVerticesToDatabase(
      const pose_graph::VertexIdList& vertices, const bool merge_landmarks,
      const bool add_lc_edges, vi_map::VIMap* map,
      pose::Transformation* T_G_M_estimate,
      vi_map::LoopClosureConstraintVector* inlier_constraints) const;

  void instantiateVisualizer();

  void clear();

  std::string printStatus() const;

 private:
  typedef std::vector<size_t> SupsampledToFullIndexMap;
  typedef std::unordered_map<loop_closure::KeyframeId, SupsampledToFullIndexMap>
      KeyframeToKeypointReindexMap;

  void findNearestNeighborMatchesForNFrame(
      const aslam::VisualNFrame& n_frame, const bool skip_untracked_keypoints,
      std::vector<vi_map::LandmarkIdList>* query_vertex_landmark_ids,
      unsigned int* num_of_lc_matches,
      loop_closure::FrameToMatches* frame_matches_list) const;

  void convertFrameToProjectedImage(
      const vi_map::VIMap& map, const vi_map::VisualFrameIdentifier& frame_id,
      const aslam::VisualFrame& frame,
      const vi_map::LandmarkIdList& observed_landmark_ids,
      const vi_map::MissionId& mission_id, const bool skip_invalid_landmark_ids,
      loop_closure::ProjectedImage* projected_image) const;

  void convertFrameToProjectedImageOnlyUsingProvidedLandmarkIds(
      const vi_map::VIMap& map, const vi_map::VisualFrameIdentifier& frame_id,
      const aslam::VisualFrame& frame,
      const vi_map::LandmarkIdList& observed_landmark_ids,
      const vi_map::MissionId& mission_id, const bool skip_invalid_landmark_ids,
      const vi_map::LandmarkIdSet& landmarks_to_add,
      loop_closure::ProjectedImage* projected_image) const;

  // A localization frame generates fake landmark ids so that the same
  // interfaces can be used as for loop-closure.
  void convertLocalizationFrameToProjectedImage(
      const aslam::VisualNFrame& nframe,
      const loop_closure::KeyframeId& keyframe_id,
      const bool skip_untracked_keypoints,
      const loop_closure::ProjectedImage::Ptr& projected_image,
      KeyframeToKeypointReindexMap* keyframe_to_keypoint_reindexing,
      vi_map::LandmarkIdList* observed_landmark_ids) const;

  bool handleLoopClosures(
      const vi_map::LoopClosureConstraint& constraint,
      const bool merge_landmarks, const bool add_lc_edges, int* num_inliers,
      double* inlier_ratio, vi_map::VIMap* map,
      pose::Transformation* T_G_I_ransac,
      vi_map::LoopClosureConstraint* inlier_constraints,
      loop_closure_handler::LoopClosureHandler::MergedLandmark3dPositionVector*
          landmark_pairs_merged,
      pose_graph::VertexId* vertex_id_closest_to_structure_matches,
      std::mutex* map_mutex) const;

  bool convertFrameMatchesToConstraint(
      const loop_closure::FrameIdMatchesPair& query_frame_id_and_matches,
      vi_map::LoopClosureConstraint* constraint_ptr) const;

  bool computeAbsoluteTransformFromFrameMatches(
      const loop_closure::FrameToMatches& frame_to_matches,
      const bool merge_landmarks, const bool add_lc_edges, vi_map::VIMap* map,
      pose::Transformation* T_G_I,
      vi_map::LoopClosureConstraint* inlier_constraints,
      pose_graph::VertexId* vertex_id_closest_to_structure_matches) const;

  bool computeAbsoluteTransformFromFrameMatches(
      const aslam::VisualNFrame& query_vertex_n_frame,
      const std::vector<vi_map::LandmarkIdList>& query_vertex_landmark_ids,
      const loop_closure::FrameToMatches& frame_to_matches,
      const bool merge_landmarks, const bool add_lc_edges,
      const loop_closure_handler::LoopClosureHandler& handler,
      pose::Transformation* T_G_I,
      vi_map::VertexKeyPointToStructureMatchList* inlier_structure_matches,
      pose_graph::VertexId* vertex_id_closest_to_structure_matches) const;

  void queryVertexInDatabase(
      const pose_graph::VertexId& query_vertex_id, const bool merge_landmarks,
      const bool add_lc_edges, vi_map::VIMap* map,
      vi_map::LoopClosureConstraint* raw_constraint,
      vi_map::LoopClosureConstraint* inlier_constraint,
      std::vector<double>* inlier_counts,
      aslam::TransformationVector* T_G_M2_vector,
      loop_closure_handler::LoopClosureHandler::MergedLandmark3dPositionVector*
          landmark_pairs_merged,
      std::mutex* map_mutex) const;

  loop_closure_visualization::LoopClosureVisualizer::UniquePtr visualizer_;
  std::shared_ptr<matching_based_loopclosure::LoopDetector> loop_detector_;
  vi_map::MissionIdSet missions_in_database_;
  summary_map::LocalizationSummaryMapIdSet summary_maps_in_database_;
  // The filename of the serialization file.
  static const std::string serialization_filename_;
  const bool use_random_pnp_seed_;

  // A mapping from the merged landmark id (does not exist anymore) to the
  // landmark id it was merged into (and should exist).
  mutable LandmarkToLandmarkMap landmark_id_old_to_new_;

  // Feature type this node deals with
  int feature_type_;
};

}  // namespace loop_detector_node

#endif  // LOOP_CLOSURE_HANDLER_LOOP_DETECTOR_NODE_H_
