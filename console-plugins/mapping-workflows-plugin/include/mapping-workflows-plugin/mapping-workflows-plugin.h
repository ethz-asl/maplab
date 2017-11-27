#ifndef MAPPING_WORKFLOWS_PLUGIN_MAPPING_WORKFLOWS_PLUGIN_H_
#define MAPPING_WORKFLOWS_PLUGIN_MAPPING_WORKFLOWS_PLUGIN_H_

#include <string>

#include <console-common/console-plugin-base-with-plotter.h>
#include <console-common/console.h>

namespace mapping_workflows_plugin {
class MappingToolsPlugin : public common::ConsolePluginBaseWithPlotter {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit MappingToolsPlugin(
      common::Console* console, visualization::ViwlsGraphRvizPlotter* plotter);

  virtual std::string getPluginId() const {
    return "mapping_workflows";
  }
};
}  // namespace mapping_workflows_plugin
#endif  // MAPPING_WORKFLOWS_PLUGIN_MAPPING_WORKFLOWS_PLUGIN_H_
