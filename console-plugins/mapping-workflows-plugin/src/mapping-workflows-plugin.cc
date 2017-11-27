#include "mapping-workflows-plugin/mapping-workflows-plugin.h"

#include <string>
#include <vector>

#include <console-common/console.h>
#include <map-manager/map-manager.h>
#include <map-sparsification/keyframe-pruning.h>
#include <vi-map/vi-map.h>
#include <visualization/viwls-graph-plotter.h>

#include "mapping-workflows-plugin/localization-map-creation.h"

DECLARE_string(map_mission);

namespace mapping_workflows_plugin {

MappingToolsPlugin::MappingToolsPlugin(
    common::Console* console, visualization::ViwlsGraphRvizPlotter* plotter)
    : common::ConsolePluginBaseWithPlotter(console, plotter) {
  addCommand(
      {"create_localization_summary_map", "clsm"},
      [plotter, this]() -> int {
        // Select map.
        std::string selected_map_key;
        if (!getSelectedMapKeyIfSet(&selected_map_key)) {
          return common::kStupidUserError;
        }
        vi_map::VIMapManager map_manager;
        vi_map::VIMapManager::MapWriteAccess map =
            map_manager.getMapWriteAccess(selected_map_key);

        // Select mission.
        vi_map::MissionId mission_id;
        map->ensureMissionIdValid(FLAGS_map_mission, &mission_id);
        if (!mission_id.isValid()) {
          LOG(ERROR) << "The given mission \"" << FLAGS_map_mission
                     << "\" is not valid.";
          return common::kStupidUserError;
        }

        map_sparsification::KeyframingHeuristicsOptions keyframe_options =
            map_sparsification::KeyframingHeuristicsOptions::
                initializeFromGFlags();
        constexpr bool kInitLandmarks = true;
        return processVIMapToLocalizationMap(
            kInitLandmarks, keyframe_options, map.get(), plotter);
      },
      "Process loaded mission to localization map.", common::Processing::Sync);
}

}  // namespace mapping_workflows_plugin

MAPLAB_CREATE_CONSOLE_PLUGIN_WITH_PLOTTER(
    mapping_workflows_plugin::MappingToolsPlugin);
