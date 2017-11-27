#ifndef LOOP_CLOSURE_PLUGIN_LOOP_CLOSURE_PLUGIN_H_
#define LOOP_CLOSURE_PLUGIN_LOOP_CLOSURE_PLUGIN_H_

#include <string>

#include <console-common/console-plugin-base-with-plotter.h>

namespace common {
class Console;
}  // namespace common

namespace visualization {
class ViwlsGraphRvizPlotter;
}  // namespace visualization

namespace loop_closure_plugin {
class LoopClosurePlugin : public common::ConsolePluginBaseWithPlotter {
 public:
  LoopClosurePlugin(
      common::Console* console, visualization::ViwlsGraphRvizPlotter* plotter);

  virtual std::string getPluginId() const {
    return "loopclosure";
  }

 private:
  int findLoopClosuresBetweenAllMissions() const;
  int findLoopClosuresInOneMission() const;
  int deleteAllLoopClosureEdges() const;
  int serializeLoopDetector() const;
  int alignMissionsForEvaluation() const;
  int evaluateLocalization() const;
};
}  // namespace loop_closure_plugin

#endif  // LOOP_CLOSURE_PLUGIN_LOOP_CLOSURE_PLUGIN_H_
