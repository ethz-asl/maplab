#ifndef MAP_SPARSIFICATION_OPTIMIZATION_LP_SOLVE_SPARSIFICATION_H_
#define MAP_SPARSIFICATION_OPTIMIZATION_LP_SOLVE_SPARSIFICATION_H_

#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <vi-map/unique-id.h>
#include <vi-map/vi-map.h>

#include "map-sparsification/sampler-base.h"

namespace map_sparsification {

// Forward declaration of a wrapper that contains a pointer to a lp_solve
// lprec structure. By using it, we avoid populating all the
// global namespace definitions of lp_solve to our project.
struct LprecWrapper;

class LpSolveSparsification : public SamplerBase {
 public:
  typedef std::unordered_map<vi_map::LandmarkId, unsigned int>
      StoreLandmarkIdToIndexMap;

  explicit LpSolveSparsification(unsigned int min_keypoints_per_keyframe)
      : num_variables_(0u),
        min_keypoints_per_keyframe_(min_keypoints_per_keyframe) {}

  virtual void sampleMapSegment(
      const vi_map::VIMap& map, unsigned int desired_num_landmarks,
      unsigned int time_limit_seconds,
      const vi_map::LandmarkIdSet& segment_store_landmark_id_set,
      const pose_graph::VertexIdList& segment_vertex_id_list,
      vi_map::LandmarkIdSet* summary_store_landmark_ids);

  virtual std::string getTypeString() const {
    return "lp_solve_ilp";
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  void addKeyframeConstraint(
      unsigned int min_keypoints_per_keyframe,
      const vi_map::LandmarkIdSet& keyframe_landmarks,
      const LprecWrapper& lprec_ptr) const;
  void addTotalLandmarksConstraint(
      unsigned int desired_num_landmarks, const LprecWrapper& lprec_ptr) const;
  void addObjectiveFunction(
      const vi_map::VIMap& map, const LprecWrapper& lprec_ptr) const;
  void setLandmarkSwitchVariablesToBinary(const LprecWrapper& lprec_ptr) const;

  vi_map::VIMap map_;
  StoreLandmarkIdToIndexMap landmark_ids_to_indices_;
  unsigned int num_variables_;
  unsigned int min_keypoints_per_keyframe_;
};

}  // namespace map_sparsification
#endif  // MAP_SPARSIFICATION_OPTIMIZATION_LP_SOLVE_SPARSIFICATION_H_
