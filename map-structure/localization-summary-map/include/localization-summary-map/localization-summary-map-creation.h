#ifndef LOCALIZATION_SUMMARY_MAP_LOCALIZATION_SUMMARY_MAP_CREATION_H_
#define LOCALIZATION_SUMMARY_MAP_LOCALIZATION_SUMMARY_MAP_CREATION_H_

#include <vi-map/unique-id.h>

namespace vi_map {
class VIMap;
}  // namespace vi_map

namespace summary_map {
class LocalizationSummaryMap;
class LocalizationSummaryMapCache;

void createLocalizationSummaryMapForWellConstrainedLandmarks(
    const vi_map::VIMap& map, summary_map::LocalizationSummaryMap* summary_map);

void createLocalizationSummaryMapForSummarizedLandmarks(
    const vi_map::VIMap& map, const double landmark_keep_fraction,
    summary_map::LocalizationSummaryMap* summary_map);

void createLocalizationSummaryMapFromLandmarkList(
    const vi_map::VIMap& map, const vi_map::LandmarkIdList& landmark_ids,
    summary_map::LocalizationSummaryMap* summary_map);

void createLocalizationSummaryMapFromLandmarkList(
    const vi_map::VIMap& map, const vi_map::LandmarkIdList& landmark_ids,
    LocalizationSummaryMapCache* summary_map_cache,
    summary_map::LocalizationSummaryMap* summary_map);

}  // namespace summary_map
#endif  // LOCALIZATION_SUMMARY_MAP_LOCALIZATION_SUMMARY_MAP_CREATION_H_
