#ifndef LOOP_CLOSURE_HANDLER_LOOP_CLOSURE_HANDLER_H_
#define LOOP_CLOSURE_HANDLER_LOOP_CLOSURE_HANDLER_H_

#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glog/logging.h>
#include <gtest/gtest_prod.h>
#include <localization-summary-map/localization-summary-map.h>
#include <vi-map-helpers/vi-map-nearest-neighbor-lookup.h>
#include <vi-map/landmark.h>
#include <vi-map/loop-constraint.h>
#include <vi-map/mission.h>
#include <vi-map/pose-graph.h>
#include <vi-map/unique-id.h>
#include <vi-map/vertex.h>
#include <vi-map/vi-map.h>
#include <vi-map/viwls-edge.h>

#include "loop-closure-handler/loop-closure-constraint.h"

DECLARE_double(lc_ransac_pixel_sigma);

namespace loop_closure_handler {

class LoopClosureHandler {
 public:
  typedef std::unordered_map<FrameKeypointIndexPair,
                             vi_map::LandmarkIdSet> KeypointToLandmarksMap;
  typedef std::vector<
      std::pair<FrameKeypointIndexPair, vi_map::LandmarkId>>
      KeypointToLandmarkVector;

  typedef std::vector<
      std::pair<vi_map::LandmarkId, vi_map::LandmarkId> >
      LandmarkToLandmarkVector;
  typedef std::unordered_map<vi_map::LandmarkId, vi_map::LandmarkId>
      LandmarkToLandmarkMap;

  typedef std::pair<Eigen::Vector3d, Eigen::Vector3d> Vector3dPair;
  typedef Aligned<std::vector, Vector3dPair> MergedLandmark3dPositionVector;

  typedef vi_map::MissionBaseFrameMap MissionBaseFrameMap;

  friend class LoopClosureHandlerTest;

  explicit LoopClosureHandler(vi_map::VIMap* map,
                              LandmarkToLandmarkMap* landmark_id_old_to_new);

  explicit LoopClosureHandler(
      summary_map::LocalizationSummaryMap const* summary_map,
      LandmarkToLandmarkMap* landmark_id_old_to_new);

  LoopClosureHandler() = delete;

  // Handle matches from one vertex to the map.
  // @param[out]  landmark_pairs_merged  List of store landmark ID pairs of
  //                                     landmarks that are considered the
  //                                     same.
  bool handleLoopClosure(
      const vi_map::LoopClosureConstraint& loop_closure_constraint,
      bool merge_matching_landmarks, bool add_loopclosure_edges,
      int* num_inliers, double* inlier_ratio,
      pose::Transformation* T_G_I_ransac,
      vi_map::LoopClosureConstraint* inlier_constraints,
      MergedLandmark3dPositionVector* landmark_pairs_merged,
      pose_graph::VertexId* vertex_id_closest_to_structure_matches,
      std::mutex* map_mutex, bool use_random_pnp_seed = true) const;

  bool handleLoopClosure(
      const aslam::VisualNFrame& query_vertex_n_frame,
      const std::vector<vi_map::LandmarkIdList>& query_vertex_landmark_ids,
      const pose_graph::VertexId& query_vertex_id,
      const vi_map::VertexKeyPointToStructureMatchList& structure_matches,
      bool merge_matching_landmarks, bool add_loopclosure_edges,
      int* num_inliers, double* inlier_ratio,
      pose::Transformation* T_G_I_ransac,
      vi_map::VertexKeyPointToStructureMatchList* inlier_structure_matches,
      MergedLandmark3dPositionVector* landmark_pairs_merged,
      pose_graph::VertexId* vertex_id_closest_to_structure_matches,
      std::mutex* map_mutex, bool use_random_pnp_seed = true) const;

  void updateQueryKeyframeInvalidLandmarkAssociations(
      const std::vector<int>& inliers,
      const KeypointToLandmarkVector& query_keypoint_idx_to_landmark_pairs,
      vi_map::Vertex* query_vertex) const;

  void mergeLandmarks(
      const std::vector<int>& inliers,
      const LandmarkToLandmarkVector& query_landmark_to_map_landmark_pairs,
      MergedLandmark3dPositionVector* landmark_pairs_actually_merged) const;

 private:
  inline Eigen::Vector3d getLandmark_p_G_fi(
      const vi_map::LandmarkId landmark_id) const {
    if (map_ != nullptr) {
      return map_->getLandmark_G_p_fi(landmark_id);
    }

    CHECK(summary_map_ != nullptr);
    return summary_map_->getGLandmarkPosition(landmark_id);
  }

  inline vi_map::LandmarkId getLandmarkIdAfterMerges(
      const vi_map::LandmarkId& old_landmark_id) const {
    CHECK_NOTNULL(landmark_id_old_to_new_);
    vi_map::LandmarkId landmark_id = old_landmark_id;

    do {
      LandmarkToLandmarkMap::iterator it_map_landmark_already_changed =
          landmark_id_old_to_new_->find(landmark_id);
      if (it_map_landmark_already_changed != landmark_id_old_to_new_->end()) {
        // Recursively update the landmark_id according to the past merges.
        landmark_id = it_map_landmark_already_changed->second;
      }
    } while (landmark_id_old_to_new_->count(landmark_id) > 0);

    return landmark_id;
  }

  vi_map::VIMap* map_;
  summary_map::LocalizationSummaryMap const* summary_map_;
  LandmarkToLandmarkMap* landmark_id_old_to_new_;
};
}  // namespace loop_closure_handler

#endif  // LOOP_CLOSURE_HANDLER_LOOP_CLOSURE_HANDLER_H_
