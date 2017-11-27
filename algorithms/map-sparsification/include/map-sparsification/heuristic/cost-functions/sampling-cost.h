#ifndef MAP_SPARSIFICATION_HEURISTIC_COST_FUNCTIONS_SAMPLING_COST_H_
#define MAP_SPARSIFICATION_HEURISTIC_COST_FUNCTIONS_SAMPLING_COST_H_

#include <unordered_map>

#include <maplab-common/macros.h>
#include <vi-map/unique-id.h>
#include <vi-map/vi-map.h>

namespace map_sparsification {
namespace cost_functions {

class SamplingCostFunction {
 public:
  MAPLAB_POINTER_TYPEDEFS(SamplingCostFunction);

  typedef std::unordered_map<pose_graph::VertexId, unsigned int>
      KeypointPerVertexCountMap;

  SamplingCostFunction() : weight_(1.0) {
    loss_function_ = [](double x) { return x; };  // NOLINT
  }
  virtual ~SamplingCostFunction() {}

  double operator()(
      const vi_map::LandmarkId& landmark_id, const vi_map::VIMap& map,
      const KeypointPerVertexCountMap& keyframe_keypoint_counts) const {
    double raw_score = scoreImpl(landmark_id, map, keyframe_keypoint_counts);
    return weight_ * loss_function_(raw_score);
  }

  void setWeight(double weight) {
    weight_ = weight;
  }

  void setLossFunction(
      const std::function<double(double)>& loss_function) {  // NOLINT
    loss_function_ = loss_function;
  }

 private:
  virtual double scoreImpl(
      const vi_map::LandmarkId& store_landmark_id, const vi_map::VIMap& map,
      const KeypointPerVertexCountMap& keyframe_keypoint_counts) const = 0;
  double weight_;
  std::function<double(double)> loss_function_;  // NOLINT
};

}  // namespace cost_functions
}  // namespace map_sparsification
#endif  // MAP_SPARSIFICATION_HEURISTIC_COST_FUNCTIONS_SAMPLING_COST_H_
