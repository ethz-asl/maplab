#include "vi-map-summarization-plugin/summarization-plugin.h"

#include <console-common/console.h>
#include <localization-summary-map/localization-summary-map-creation.h>
#include <localization-summary-map/localization-summary-map.h>
#include <map-manager/map-manager.h>
#include <maplab-common/file-system-tools.h>
#include <vi-map/vi-map.h>

DEFINE_string(summary_map_save_path, "", "Save path of the summary map.");
DECLARE_bool(overwrite);

namespace summarization_plugin {

SummarizationPlugin::SummarizationPlugin(common::Console* console)
    : common::ConsolePluginBase(CHECK_NOTNULL(console)) {
  addCommand(
      {"generate_summary_map_and_save_to_disk", "summary_map"},
      [this]() -> int { return saveSummaryMapToDisk(); },
      "Generate a summary map of the selected map and save it to the path "
      "given by --summary_map_save_path.",
      common::Processing::Sync);
}

int SummarizationPlugin::saveSummaryMapToDisk() const {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }

  if (FLAGS_summary_map_save_path.empty()) {
    LOG(ERROR) << "No save path for the summary map has been provided. Use "
               << "--summary_map_save_path to set one.";
    return common::kStupidUserError;
  }

  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapReadAccess map =
      map_manager.getMapReadAccess(selected_map_key);
  summary_map::LocalizationSummaryMap summary_map;
  summary_map::createLocalizationSummaryMapForWellConstrainedLandmarks(
      *map, &summary_map);

  backend::SaveConfig save_config;
  save_config.overwrite_existing_files = FLAGS_overwrite;
  if (!summary_map.saveToFolder(FLAGS_summary_map_save_path, save_config)) {
    LOG(ERROR) << "Saving summary map failed.";
    return common::kUnknownError;
  }

  return common::kSuccess;
}

}  // namespace summarization_plugin

MAPLAB_CREATE_CONSOLE_PLUGIN(summarization_plugin::SummarizationPlugin);
