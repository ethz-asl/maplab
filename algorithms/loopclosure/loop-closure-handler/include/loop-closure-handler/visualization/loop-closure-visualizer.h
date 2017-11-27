#ifndef VISUALIZATION_LOOP_CLOSURE_VISUALIZER_H_
#define VISUALIZATION_LOOP_CLOSURE_VISUALIZER_H_

#include <string>

#include <vi-map/vi-map.h>
#include <visualization/viwls-graph-plotter.h>

#include "loop-closure-handler/loop-closure-constraint.h"
#include "loop-closure-handler/loop-closure-handler.h"

namespace loop_closure_visualization {

class LoopClosureVisualizer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MAPLAB_POINTER_TYPEDEFS(LoopClosureVisualizer);

  typedef std::unordered_map<vi_map::LandmarkId, vi_map::LandmarkId>
      LandmarkToLandmarkMap;

  LoopClosureVisualizer();
  virtual ~LoopClosureVisualizer();

  void visualizeMergedLandmarks(
      const loop_closure_handler::LoopClosureHandler::
          MergedLandmark3dPositionVector& landmark_pairs_merged);

  // Merged landmark map is necessary, as over the course of loopclosing
  // some landmarks may have been merged and the landmark ids stored in the
  // constraints might no longer be present in the vi-map.
  void visualizeKeyframeToStructureMatches(
      const vi_map::LoopClosureConstraintVector& inlier_constraints,
      const vi_map::LoopClosureConstraintVector& all_constraints,
      const LandmarkToLandmarkMap& merged_landmark_map,
      const vi_map::VIMap& map);

  void visualizeKeyframeToStructureMatch(
      const vi_map::VertexKeyPointToStructureMatchList& structure_matches,
      const Eigen::Vector3d& query_position,
      const summary_map::LocalizationSummaryMap& localization_summary_map);

  void visualizeFullMapDatabase(
      const vi_map::MissionIdList& missions, const vi_map::VIMap& map);

  void visualizeSummaryMapDatabase(
      const summary_map::LocalizationSummaryMap& localization_summary_map);

 private:
  void addKeyframeToStructureMatchMarker(
      const vi_map::LoopClosureConstraint& constraint,
      const visualization::Color& color,
      const LandmarkToLandmarkMap& merged_landmark_map,
      const vi_map::VIMap& map, visualization::LineSegmentVector* matches,
      vi_map::LandmarkIdSet* added_landmarks);

  const std::string loop_closures_topic_;
  const std::string landmark_pairs_topic_;

  const visualization::ViwlsGraphRvizPlotter vi_map_plotter_;
};

}  // namespace loop_closure_visualization

#endif  // VISUALIZATION_LOOP_CLOSURE_VISUALIZER_H_
