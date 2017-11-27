#include "loop-closure-handler/inlier-index-with-reprojection-error.h"

#include <glog/logging.h>

namespace loop_closure_handler {

void getBestStructureMatchForEveryKeypoint(
    const std::vector<int>& inliers,
    const std::vector<double>& inlier_distances_to_model,
    const vi_map::VertexKeyPointToStructureMatchList& structure_matches,
    const aslam::VisualNFrame& query_vertex_n_frame,
    KeypointToInlierIndexWithReprojectionErrorMap*
        keypoint_to_best_structure_match) {
  CHECK_NOTNULL(keypoint_to_best_structure_match)->clear();
  CHECK_EQ(inliers.size(), inlier_distances_to_model.size());

  pose_graph::VertexId invalid_vertex_id;
  for (size_t inlier_idx = 0u; inlier_idx < inliers.size(); ++inlier_idx) {
    const int inlier_index = inliers[inlier_idx];
    const double reprojection_error = inlier_distances_to_model[inlier_idx];
    const size_t frame_index =
        structure_matches[inlier_index].frame_index_query;
    CHECK_LT(frame_index, query_vertex_n_frame.getNumFrames());
    const size_t keypoint_index =
        structure_matches[inlier_index].keypoint_index_query;
    CHECK_LT(
        keypoint_index, query_vertex_n_frame.getFrame(frame_index)
                            .getNumKeypointMeasurements());
    const vi_map::KeypointIdentifier keypoint_identifier(
        invalid_vertex_id, frame_index, keypoint_index);

    KeypointToInlierIndexWithReprojectionErrorMap::iterator
        best_match_iterator =
            keypoint_to_best_structure_match->find(keypoint_identifier);
    if (best_match_iterator == keypoint_to_best_structure_match->end()) {
      InlierIndexWithReprojectionError inlier_index_with_reprojection_error(
          inlier_index, reprojection_error);
      CHECK(
          keypoint_to_best_structure_match
              ->emplace(
                  keypoint_identifier, inlier_index_with_reprojection_error)
              .second);
    } else if (
        best_match_iterator->second.getReprojectionError() >
        reprojection_error) {
      // Replace by new best structure-match.
      best_match_iterator->second.setReprojectionError(reprojection_error);
      best_match_iterator->second.setInlierIndex(inlier_index);
    }
  }
}

}  // namespace loop_closure_handler
