#include "localization-summary-map/localization-summary-map-queries.h"

#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>
#include <glog/logging.h>
#include <maplab-common/accessors.h>
#include <maplab-common/geometry.h>
#include <vi-map/landmark.h>

#include "localization-summary-map/localization-summary-map.h"

namespace summary_map {

SummaryMapCachedLookups::SummaryMapCachedLookups(
    const LocalizationSummaryMap& map)
    : map_(map) {
  buildLandmarkIdToObserverIndicesMap();
}

const std::unordered_set<size_t>&
SummaryMapCachedLookups::getAllObserverIndicesForLandmark(
    const vi_map::LandmarkId& landmark_id) const {
  return common::getChecked(landmarkid_observerindices_, landmark_id);
}

void SummaryMapCachedLookups::buildLandmarkIdToObserverIndicesMap() {
  const Eigen::Matrix<unsigned int, Eigen::Dynamic, 1>&
      observation_to_landmark_index = map_.observationToLandmarkIndex();
  const Eigen::Matrix<unsigned int, Eigen::Dynamic, 1>&
      observation_to_observer_index = map_.observerIndices();
  vi_map::LandmarkIdList landmark_ids;
  map_.getAllLandmarkIds(&landmark_ids);

  for (int oidx = 0u; oidx < observation_to_landmark_index.rows(); ++oidx) {
    CHECK_LT(oidx, observation_to_landmark_index.rows());
    const size_t landmark_idx = observation_to_landmark_index(oidx, 0);
    CHECK_LT(oidx, observation_to_observer_index.rows());
    const size_t observer_idx = observation_to_observer_index(oidx, 0);
    CHECK_LT(landmark_idx, landmark_ids.size());
    const vi_map::LandmarkId landmark_id = landmark_ids[landmark_idx];
    landmarkid_observerindices_[landmark_id].emplace(observer_idx);
  }
}

double getMaxDisparityRadAngleOfLandmarkObservationBundle(
    const LocalizationSummaryMap& map,
    const SummaryMapCachedLookups& map_lookup_cache,
    const vi_map::LandmarkId& landmark_id) {
  CHECK(landmark_id.isValid());

  // Build landmark observation bundle.
  const Eigen::Matrix3Xf& G_observers = map.GObserverPosition();

  const std::unordered_set<size_t>& observer_indices =
      map_lookup_cache.getAllObserverIndicesForLandmark(landmark_id);
  CHECK(!observer_indices.empty());

  Aligned<std::vector, Eigen::Vector3d> G_normalized_incidence_rays;
  G_normalized_incidence_rays.resize(observer_indices.size());

  size_t idx = 0u;
  for (size_t observer_idx : observer_indices) {
    const Eigen::Vector3d G_landmark = map.getGLandmarkPosition(landmark_id);
    CHECK_LT(static_cast<int>(observer_idx), G_observers.cols());
    const Eigen::Vector3d G_observer =
        G_observers.col(observer_idx).cast<double>();

    const Eigen::Vector3d G_incidence_ray = G_observer - G_landmark;
    G_normalized_incidence_rays[idx] = G_incidence_ray.normalized();
    ++idx;
  }

  // Build bundle of incidence rays to the landmark.
  return common::getMaxDisparityRadAngleOfUnitVectorBundle(
      G_normalized_incidence_rays);
}

// Consider building the required lookup externally for efficiency.
double getMaxDisparityRadAngleOfLandmarkObservationBundle(
    const LocalizationSummaryMap& map, const vi_map::LandmarkId& id) {
  SummaryMapCachedLookups map_lookup_cache(map);
  return getMaxDisparityRadAngleOfLandmarkObservationBundle(
      map, map_lookup_cache, id);
}

}  // namespace summary_map
