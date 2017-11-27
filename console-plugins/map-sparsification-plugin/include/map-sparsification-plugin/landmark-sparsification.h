#ifndef MAP_SPARSIFICATION_PLUGIN_LANDMARK_SPARSIFICATION_H_
#define MAP_SPARSIFICATION_PLUGIN_LANDMARK_SPARSIFICATION_H_

#include <vi-map/vi-map.h>

namespace map_sparsification_plugin {

bool sparsifyMapLandmarks(
    unsigned int num_landmarks_to_keep, vi_map::VIMap* map);

}  // namespace map_sparsification_plugin

#endif  // MAP_SPARSIFICATION_PLUGIN_LANDMARK_SPARSIFICATION_H_
