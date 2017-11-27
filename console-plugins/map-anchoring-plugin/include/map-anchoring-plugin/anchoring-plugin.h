#ifndef MAP_ANCHORING_PLUGIN_ANCHORING_PLUGIN_H_
#define MAP_ANCHORING_PLUGIN_ANCHORING_PLUGIN_H_

#include <string>

#include <console-common/console-plugin-base-with-plotter.h>
#include <console-common/console.h>
#include <vi-map/vi-map.h>

namespace map_anchoring_plugin {

class AnchoringPlugin : public common::ConsolePluginBaseWithPlotter {
 public:
  AnchoringPlugin(
      common::Console* console, visualization::ViwlsGraphRvizPlotter* plotter);

  virtual std::string getPluginId() const {
    return "anchoring";
  }

 private:
  int setMissionBaseframeKnownState(const bool baseframe_known_state) const;
  int setAllMissionBaseframesKnownState(const bool baseframe_known_state) const;

  int anchorMission() const;
  int anchorAllMissions() const;
};

}  // namespace map_anchoring_plugin

#endif  // MAP_ANCHORING_PLUGIN_ANCHORING_PLUGIN_H_
