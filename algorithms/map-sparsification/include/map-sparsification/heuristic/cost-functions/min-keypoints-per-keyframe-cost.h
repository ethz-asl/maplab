#ifndef MAP_SPARSIFICATION_HEURISTIC_COST_FUNCTIONS_MIN_KEYPOINTS_PER_KEYFRAME_COST_H_
#define MAP_SPARSIFICATION_HEURISTIC_COST_FUNCTIONS_MIN_KEYPOINTS_PER_KEYFRAME_COST_H_

#include <map-sparsification/heuristic/cost-functions/sampling-cost.h>
#include <vi-map/unique-id.h>
#include <vi-map/vi-map.h>

namespace map_sparsification {
namespace cost_functions {

class IsRequiredToConstrainKeyframesCost : public SamplingCostFunction {
 public:
  MAPLAB_POINTER_TYPEDEFS(IsRequiredToConstrainKeyframesCost);
  explicit IsRequiredToConstrainKeyframesCost(int min_keypoints_per_keyframe)
      : min_keypoints_per_keyframe_(min_keypoints_per_keyframe) {}
  virtual ~IsRequiredToConstrainKeyframesCost() {}

 private:
  virtual inline double scoreImpl(
      const vi_map::LandmarkId& landmark_id, const vi_map::VIMap& map,
      const KeypointPerVertexCountMap& keyframe_keypoint_counts) const {
    double total_keyframe_cost = 0.0;

    // Verify if each of the observer frames has already enough landmarks.
    const vi_map::Landmark& landmark = map.getLandmark(landmark_id);
    for (const vi_map::KeypointIdentifier& observation :
         landmark.getObservations()) {
      const pose_graph::VertexId& vertex_id = observation.frame_id.vertex_id;
      KeypointPerVertexCountMap::const_iterator keypoint_count_it =
          keyframe_keypoint_counts.find(vertex_id);

      // Count only observer frames from the current segment.
      if (keypoint_count_it != keyframe_keypoint_counts.end()) {
        total_keyframe_cost += std::max(
            0.0, static_cast<double>(
                     min_keypoints_per_keyframe_ -
                     static_cast<int>(keypoint_count_it->second)));
      }
    }
    return total_keyframe_cost;
  }

  int min_keypoints_per_keyframe_;
};

}  // namespace cost_functions
}  // namespace map_sparsification
#endif  // MAP_SPARSIFICATION_HEURISTIC_COST_FUNCTIONS_MIN_KEYPOINTS_PER_KEYFRAME_COST_H_
