#ifndef MAP_SPARSIFICATION_HEURISTIC_HEURISTIC_SAMPLING_H_
#define MAP_SPARSIFICATION_HEURISTIC_HEURISTIC_SAMPLING_H_

#include <functional>
#include <string>
#include <utility>
#include <vector>

#include <maplab-common/macros.h>
#include <vi-map/unique-id.h>
#include <vi-map/vi-map.h>

#include "map-sparsification/heuristic/cost-functions/sampling-cost.h"
#include "map-sparsification/heuristic/scoring/scoring-function.h"
#include "map-sparsification/sampler-base.h"

namespace map_sparsification {
namespace sampling {

class LandmarkSamplingWithCostFunctions : public SamplerBase {
 public:
  MAPLAB_POINTER_TYPEDEFS(LandmarkSamplingWithCostFunctions);

  typedef map_sparsification::cost_functions::SamplingCostFunction
      SamplingCostFunction;
  typedef map_sparsification::scoring::ScoringFunction ScoringFunction;
  typedef SamplingCostFunction::KeypointPerVertexCountMap
      KeyframeKeypointCountMap;

  typedef std::pair<vi_map::LandmarkId, double> StoreLandmarkIdScorePair;
  typedef std::unordered_map<vi_map::LandmarkId, double> LandmarkScoreMap;

  void registerScoringFunction(const ScoringFunction::ConstPtr& scoring);
  void registerCostFunction(const SamplingCostFunction::ConstPtr& cost);

  virtual void sampleMapSegment(
      const vi_map::VIMap& map, unsigned int desired_num_landmarks,
      unsigned int time_limit_seconds,
      const vi_map::LandmarkIdSet& segment_store_landmark_id_set,
      const pose_graph::VertexIdList& segment_vertex_id_list,
      vi_map::LandmarkIdSet* summary_store_landmark_ids);

  virtual std::string getTypeString() const {
    return "greedy";
  }

 private:
  std::vector<ScoringFunction::ConstPtr> scoring_functions_;
  std::vector<SamplingCostFunction::ConstPtr> cost_functions_;
};

}  // namespace sampling
}  // namespace map_sparsification
#endif  // MAP_SPARSIFICATION_HEURISTIC_HEURISTIC_SAMPLING_H_
