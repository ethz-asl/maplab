#include "map-sparsification/heuristic/random-sampling.h"

#include <random>

namespace map_sparsification {
namespace sampling {

void RandomLandmarkSampling::sampleMapSegment(
    const vi_map::VIMap& /*map*/, unsigned int desired_num_landmarks,
    unsigned int /*time_limit_seconds*/,
    const vi_map::LandmarkIdSet& segment_landmark_id_set,
    const pose_graph::VertexIdList& /*segment_vertex_id_list*/,
    vi_map::LandmarkIdSet* summary_landmark_ids) {
  CHECK_NOTNULL(summary_landmark_ids)->clear();

  const size_t num_landmarks = segment_landmark_id_set.size();

  if (num_landmarks == 0u) {
    LOG(WARNING) << "No landmarks provided, bailing out early.";
    return;
  }

  if (num_landmarks <= desired_num_landmarks) {
    LOG(WARNING) << "Desired landmark count " << desired_num_landmarks
                 << " larger or equal to the total landmark count "
                 << num_landmarks << ", bailing out early.";
    summary_landmark_ids->insert(
        segment_landmark_id_set.begin(), segment_landmark_id_set.end());
    return;
  }

  vi_map::LandmarkIdList segment_landmark_id_list(
      segment_landmark_id_set.begin(), segment_landmark_id_set.end());

  constexpr int kSeed = 1234;
  std::mt19937 generator(kSeed);
  std::uniform_int_distribution<> distribution(0, num_landmarks - 1);

  summary_landmark_ids->reserve(desired_num_landmarks);
  while (summary_landmark_ids->size() < desired_num_landmarks) {
    const size_t index = distribution(generator);
    CHECK_LT(index, segment_landmark_id_list.size());
    summary_landmark_ids->insert(segment_landmark_id_list[index]);
  }
}

}  // namespace sampling
}  // namespace map_sparsification
