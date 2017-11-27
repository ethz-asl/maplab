#ifndef MAP_SPARSIFICATION_HEURISTIC_RANDOM_SAMPLING_H_
#define MAP_SPARSIFICATION_HEURISTIC_RANDOM_SAMPLING_H_

#include <string>

#include <maplab-common/macros.h>
#include <vi-map/unique-id.h>
#include <vi-map/vi-map.h>

#include "map-sparsification/sampler-base.h"

namespace map_sparsification {
namespace sampling {

class RandomLandmarkSampling : public SamplerBase {
 public:
  MAPLAB_POINTER_TYPEDEFS(RandomLandmarkSampling);

  virtual void sampleMapSegment(
      const vi_map::VIMap& map, unsigned int desired_num_landmarks,
      unsigned int time_limit_seconds,
      const vi_map::LandmarkIdSet& segment_store_landmark_id_set,
      const pose_graph::VertexIdList& segment_vertex_id_list,
      vi_map::LandmarkIdSet* summary_store_landmark_ids);

  virtual std::string getTypeString() const {
    return "random";
  }
};

}  // namespace sampling
}  // namespace map_sparsification
#endif  // MAP_SPARSIFICATION_HEURISTIC_RANDOM_SAMPLING_H_
