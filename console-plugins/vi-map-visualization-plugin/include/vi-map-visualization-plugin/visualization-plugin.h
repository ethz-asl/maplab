#ifndef VI_MAP_VISUALIZATION_PLUGIN_VISUALIZATION_PLUGIN_H_
#define VI_MAP_VISUALIZATION_PLUGIN_VISUALIZATION_PLUGIN_H_

#include <memory>

#include <Eigen/Dense>
#include <console-common/console-plugin-base.h>
#include <console-common/console.h>
#include <string>
#include <vi-map/vi-map.h>
#include <visualization/visualizer.h>

namespace common {
class Console;
}  // namespace common

namespace vi_visualization {

class VisualizationPlugin : public common::ConsolePluginBase {
 public:
  explicit VisualizationPlugin(common::Console* console);

  virtual std::string getPluginId() const {
    return "visualization";
  }

 private:
  void plotVIStatesOfMission(
      const vi_map::VIMap& map, const vi_map::MissionId& mission_id) const;
  int visualizerCvMatResources(backend::ResourceType type);
  std::unique_ptr<visualization::Visualizer> visualizer_;
};

}  // namespace vi_visualization

#endif  // VI_MAP_VISUALIZATION_PLUGIN_VISUALIZATION_PLUGIN_H_
