#include "map-sparsification/heuristic/no-sampling.h"

namespace map_sparsification {
namespace sampling {

void NoLandmarkSampling::sampleMapSegment(
    const vi_map::VIMap& /*map*/, unsigned int /*desired_num_landmarks*/,
    unsigned int /*time_limit_seconds*/,
    const vi_map::LandmarkIdSet& segment_landmark_id_set,
    const pose_graph::VertexIdList& /*segment_vertex_id_list*/,
    vi_map::LandmarkIdSet* summary_landmark_ids) {
  *CHECK_NOTNULL(summary_landmark_ids) = segment_landmark_id_set;
  LOG(WARNING) << "No sampling is used, will keep all the landmarks. Desired "
               << "landmark count is ignored.";
}

}  // namespace sampling
}  // namespace map_sparsification
