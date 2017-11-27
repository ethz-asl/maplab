#ifndef LOOP_CLOSURE_HANDLER_INLIER_INDEX_WITH_REPROJECTION_ERROR_H_
#define LOOP_CLOSURE_HANDLER_INLIER_INDEX_WITH_REPROJECTION_ERROR_H_

#include <unordered_map>
#include <vector>

#include <vi-map/loop-constraint.h>
#include <vi-map/unique-id.h>

namespace loop_closure_handler {

struct InlierIndexWithReprojectionError {
  InlierIndexWithReprojectionError() = delete;
  InlierIndexWithReprojectionError(
      const int inlier_index, const double reprojection_error)
      : inlier_index_(inlier_index), reprojection_error_(reprojection_error) {}
  virtual ~InlierIndexWithReprojectionError() = default;
  inline int getInlierIndex() const {
    return inlier_index_;
  }
  inline void setInlierIndex(const int inlier_index) {
    inlier_index_ = inlier_index;
  }
  inline double getReprojectionError() const {
    return reprojection_error_;
  }
  inline void setReprojectionError(const double reprojection_error) {
    reprojection_error_ = reprojection_error;
  }

 private:
  int inlier_index_;
  double reprojection_error_;
};
typedef std::unordered_map<vi_map::KeypointIdentifier,
                           InlierIndexWithReprojectionError>
    KeypointToInlierIndexWithReprojectionErrorMap;

void getBestStructureMatchForEveryKeypoint(
    const std::vector<int>& inliers,
    const std::vector<double>& inlier_distances_to_model,
    const vi_map::VertexKeyPointToStructureMatchList& structure_matches,
    const aslam::VisualNFrame& query_vertex_n_frame,
    KeypointToInlierIndexWithReprojectionErrorMap*
        keypoint_to_best_structure_match);
}  // namespace loop_closure_handler

#endif  // LOOP_CLOSURE_HANDLER_INLIER_INDEX_WITH_REPROJECTION_ERROR_H_
