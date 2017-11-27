#include "map-sparsification/sampler-base.h"

namespace map_sparsification {

void SamplerBase::sample(
    const vi_map::VIMap& map, unsigned int desired_num_landmarks,
    vi_map::LandmarkIdSet* summary_store_landmark_ids) {
  vi_map::LandmarkIdSet all_store_landmark_ids;
  map.getAllLandmarkIds(&all_store_landmark_ids);

  pose_graph::VertexIdList all_vertex_ids;
  map.getAllVertexIds(&all_vertex_ids);

  const unsigned int kGlobalTimeLimitSeconds = 30;
  sampleMapSegment(map, desired_num_landmarks, kGlobalTimeLimitSeconds,
                   all_store_landmark_ids, all_vertex_ids,
                   summary_store_landmark_ids);
}

}  // namespace map_sparsification
