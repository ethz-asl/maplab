#ifndef DENSE_MAPPING_DENSE_MAPPING_PLUGIN_H_
#define DENSE_MAPPING_DENSE_MAPPING_PLUGIN_H_

#include <string>

#include <console-common/console-plugin-base-with-plotter.h>

DECLARE_string(map_mission);
DECLARE_string(map_mission_list);

namespace common {
class Console;
}  // namespace common

namespace visualization {
class ViwlsGraphRvizPlotter;
}  // namespace visualization

namespace dense_mapping {
class DenseMappingPlugin : public common::ConsolePluginBaseWithPlotter {
 public:
  DenseMappingPlugin(
      common::Console* console, visualization::ViwlsGraphRvizPlotter* plotter);

  virtual std::string getPluginId() const {
    return "dense_mapping";
  }

 private:
  int addDenseMappingConstraints() const;
};
}  // namespace dense_mapping

#endif  // DENSE_MAPPING_DENSE_MAPPING_PLUGIN_H_
