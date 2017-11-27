#ifndef MAPPING_WORKFLOWS_PLUGIN_LOCALIZATION_MAP_CREATION_H_
#define MAPPING_WORKFLOWS_PLUGIN_LOCALIZATION_MAP_CREATION_H_
#include <glog/logging.h>
#include <loop-closure-plugin/vi-map-merger.h>
#include <map-optimization-legacy/ba-optimization-options.h>
#include <map-sparsification-plugin/keyframe-pruning.h>
#include <vi-map/vi-map.h>

namespace mapping_workflows_plugin {
// Processes a raw map built by the stream-map-builder into a localization
// summary map. It runs the following steps:
// landmark init - keyframing - lc - optvi
int processVIMapToLocalizationMap(
    bool initialize_landmarks,
    const map_sparsification::KeyframingHeuristicsOptions& keyframe_options,
    vi_map::VIMap* map, visualization::ViwlsGraphRvizPlotter* plotter);
}  //  namespace mapping_workflows_plugin
#endif  // MAPPING_WORKFLOWS_PLUGIN_LOCALIZATION_MAP_CREATION_H_
