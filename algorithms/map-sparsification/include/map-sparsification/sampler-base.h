#ifndef MAP_SPARSIFICATION_SAMPLER_BASE_H_
#define MAP_SPARSIFICATION_SAMPLER_BASE_H_

#include <string>

#include <maplab-common/macros.h>
#include <vi-map/vi-map.h>

namespace map_sparsification {

class SamplerBase {
 public:
  MAPLAB_POINTER_TYPEDEFS(SamplerBase);

  enum class Type {
    // Basically selects all landmarks.
    kNoSampling = 0,
    // Randomly selects a desired number of landmarks.
    kRandom = 1,
    // Uses heuristics such as the number of landmark observations and the
    // number of landmarks observations in keyframes to select a desired number
    // of landmarks.
    kHeuristic = 2,
    kLpsolveIlp = 3,
    // If the map is too large, partition the map using METIS and the solve
    // and ILP problem.
    kLpsolvePartitionIlp = 4,
  };

  virtual ~SamplerBase() {}

  void sample(const vi_map::VIMap& map, unsigned int desired_num_landmarks,
              vi_map::LandmarkIdSet* summary_store_landmark_ids);

  virtual void sampleMapSegment(
      const vi_map::VIMap& map, unsigned int desired_num_landmarks,
      unsigned int time_limit_seconds,
      const vi_map::LandmarkIdSet& segment_store_landmark_id_set,
      const pose_graph::VertexIdList& segment_vertex_id_list,
      vi_map::LandmarkIdSet* summary_store_landmark_ids) = 0;

  virtual std::string getTypeString() const = 0;
};

}  // namespace map_sparsification
#endif  // MAP_SPARSIFICATION_SAMPLER_BASE_H_
