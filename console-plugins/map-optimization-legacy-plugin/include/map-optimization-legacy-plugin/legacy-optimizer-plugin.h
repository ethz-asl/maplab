#ifndef MAP_OPTIMIZATION_LEGACY_PLUGIN_LEGACY_OPTIMIZER_PLUGIN_H_
#define MAP_OPTIMIZATION_LEGACY_PLUGIN_LEGACY_OPTIMIZER_PLUGIN_H_

#include <string>

#include <console-common/console-plugin-base-with-plotter.h>

namespace common {
class Console;
}  // namespace common

namespace visualization {
class ViwlsGraphRvizPlotter;
}  // namespace visualization

namespace map_optimization_legacy_plugin {

class OptimizerPlugin : public common::ConsolePluginBaseWithPlotter {
 public:
  OptimizerPlugin(
      common::Console* console, visualization::ViwlsGraphRvizPlotter* plotter);

  virtual std::string getPluginId() const {
    return "map-optimization-legacy";
  }

 private:
  static constexpr bool kSignalHandlerEnabled = true;

  int optimizeVisionOnly();
  int optimizeVisualInertial();
  int optimizeOneMission();
  int relax();
};

}  // namespace map_optimization_legacy_plugin

#endif  // MAP_OPTIMIZATION_LEGACY_PLUGIN_LEGACY_OPTIMIZER_PLUGIN_H_
