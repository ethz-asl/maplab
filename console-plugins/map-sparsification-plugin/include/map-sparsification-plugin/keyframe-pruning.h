#ifndef MAP_SPARSIFICATION_PLUGIN_KEYFRAME_PRUNING_H_
#define MAP_SPARSIFICATION_PLUGIN_KEYFRAME_PRUNING_H_

#include <string>

#include <console-common/console-plugin-base-with-plotter.h>
#include <console-common/console.h>
#include <map-sparsification/keyframe-pruning.h>
#include <vi-map/vi-map.h>

namespace map_sparsification_plugin {
int keyframeMapBasedOnHeuristics(
    const map_sparsification::KeyframingHeuristicsOptions& options,
    const vi_map::MissionId& mission_id,
    visualization::ViwlsGraphRvizPlotter* plotter, vi_map::VIMap* map);

}  // namespace map_sparsification_plugin
#endif  // MAP_SPARSIFICATION_PLUGIN_KEYFRAME_PRUNING_H_
