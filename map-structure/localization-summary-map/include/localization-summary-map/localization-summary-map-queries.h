#ifndef LOCALIZATION_SUMMARY_MAP_LOCALIZATION_SUMMARY_MAP_QUERIES_H_
#define LOCALIZATION_SUMMARY_MAP_LOCALIZATION_SUMMARY_MAP_QUERIES_H_

#include <unordered_map>
#include <unordered_set>

#include <Eigen/Core>
#include <glog/logging.h>
#include <vi-map/landmark.h>

#include "localization-summary-map/localization-summary-map.h"

namespace summary_map {

class SummaryMapCachedLookups {
 public:
  explicit SummaryMapCachedLookups(const LocalizationSummaryMap& map);

  const std::unordered_set<size_t>& getAllObserverIndicesForLandmark(
      const vi_map::LandmarkId& landmark_id) const;

 private:
  void buildLandmarkIdToObserverIndicesMap();

  const LocalizationSummaryMap& map_;
  std::unordered_map<vi_map::LandmarkId, std::unordered_set<size_t>>
      landmarkid_observerindices_;
};

double getMaxDisparityRadAngleOfLandmarkObservationBundle(
    const LocalizationSummaryMap& map,
    const SummaryMapCachedLookups& map_lookup_cache,
    const vi_map::LandmarkId& landmark_id);
double getMaxDisparityRadAngleOfLandmarkObservationBundle(
    const LocalizationSummaryMap& map, const vi_map::LandmarkId& id);

}  // namespace summary_map
#endif  // LOCALIZATION_SUMMARY_MAP_LOCALIZATION_SUMMARY_MAP_QUERIES_H_
