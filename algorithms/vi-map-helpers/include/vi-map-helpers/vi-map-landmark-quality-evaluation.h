#ifndef VI_MAP_HELPERS_VI_MAP_LANDMARK_QUALITY_EVALUATION_H_
#define VI_MAP_HELPERS_VI_MAP_LANDMARK_QUALITY_EVALUATION_H_

namespace vi_map {
class VIMap;
}  // namespace vi_map

namespace vi_map_helpers {

void evaluateLandmarkQuality(vi_map::VIMap* map);
void resetLandmarkQualityToUnknown(vi_map::VIMap* map);

}  // namespace vi_map_helpers

#endif  // VI_MAP_HELPERS_VI_MAP_LANDMARK_QUALITY_EVALUATION_H_
