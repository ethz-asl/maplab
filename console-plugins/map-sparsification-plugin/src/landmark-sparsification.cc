#include "map-sparsification-plugin/landmark-sparsification.h"

#include <map-sparsification/sampler-base.h>
#include <map-sparsification/sampler-factory.h>
#include <vi-map-helpers/vi-map-queries.h>

namespace map_sparsification_plugin {

bool sparsifyMapLandmarks(
    unsigned int num_landmarks_to_keep, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);

  vi_map_helpers::VIMapQueries queries(*map);
  vi_map::LandmarkIdList well_constrained_landmarks;
  queries.getAllWellConstrainedLandmarkIds(&well_constrained_landmarks);

  // Check if we there is anything to remove.
  if (num_landmarks_to_keep >= well_constrained_landmarks.size()) {
    LOG(WARNING) << "Desired number of landmarks " << num_landmarks_to_keep
                 << " is larger than the current number of well constrained"
                 << " landmarks (" << well_constrained_landmarks.size()
                 << "). Bailing out early.";
    return false;
  }

  using map_sparsification::SamplerBase;
  SamplerBase::Ptr sampler = map_sparsification::createSampler(
      SamplerBase::Type::kLpsolvePartitionIlp);

  vi_map::LandmarkIdSet landmarks_to_keep;
  sampler->sample(*map, num_landmarks_to_keep, &landmarks_to_keep);

  vi_map::LandmarkIdSet all_landmark_ids;
  map->getAllLandmarkIds(&all_landmark_ids);
  for (const vi_map::LandmarkId& landmark_id : all_landmark_ids) {
    if (landmarks_to_keep.count(landmark_id) == 0) {
      map->removeLandmark(landmark_id);
    }
  }

  return true;
}

}  // namespace map_sparsification_plugin
