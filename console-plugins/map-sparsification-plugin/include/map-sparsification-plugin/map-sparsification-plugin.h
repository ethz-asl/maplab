#ifndef MAP_SPARSIFICATION_PLUGIN_MAP_SPARSIFICATION_PLUGIN_H_
#define MAP_SPARSIFICATION_PLUGIN_MAP_SPARSIFICATION_PLUGIN_H_

#include <string>

#include <console-common/console-plugin-base-with-plotter.h>
#include <console-common/console.h>

namespace map_sparsification_plugin {

class MapSparsificationPlugin : public common::ConsolePluginBaseWithPlotter {
 public:
  explicit MapSparsificationPlugin(
      common::Console* console, visualization::ViwlsGraphRvizPlotter* plotter);

  virtual std::string getPluginId() const override {
    return "map_sparsification";
  }
};

}  // namespace map_sparsification_plugin

#endif  // MAP_SPARSIFICATION_PLUGIN_MAP_SPARSIFICATION_PLUGIN_H_
