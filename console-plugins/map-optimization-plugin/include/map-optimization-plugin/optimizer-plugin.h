#ifndef MAP_OPTIMIZATION_PLUGIN_OPTIMIZER_PLUGIN_H_
#define MAP_OPTIMIZATION_PLUGIN_OPTIMIZER_PLUGIN_H_

#include <string>

#include <console-common/console-plugin-base-with-plotter.h>

namespace common {
class Console;
}  // namespace common

namespace visualization {
class ViwlsGraphRvizPlotter;
}  // namespace visualization

namespace map_optimization_plugin {
class OptimizerPlugin : public common::ConsolePluginBaseWithPlotter {
 public:
  OptimizerPlugin(
      common::Console* console, visualization::ViwlsGraphRvizPlotter* plotter);

  virtual std::string getPluginId() const {
    return "map-optimization";
  }

 private:
  int optimizeVisualInertial(bool visual_only, bool outlier_rejection);

  int relaxMap();
  int relaxMapMissionsSeparately();

  static constexpr bool kSignalHandlerEnabled = true;
};
}  // namespace map_optimization_plugin
#endif  // MAP_OPTIMIZATION_PLUGIN_OPTIMIZER_PLUGIN_H_
