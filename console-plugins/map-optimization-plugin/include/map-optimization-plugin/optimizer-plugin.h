#ifndef MAP_OPTIMIZATION_PLUGIN_OPTIMIZER_PLUGIN_H_
#define MAP_OPTIMIZATION_PLUGIN_OPTIMIZER_PLUGIN_H_

#include <string>

#include <console-common/console-plugin-base-with-plotter.h>
#include <vi-map-data-import-export/import-loop-closure-edges.h>
#include <vi-map/vi-map.h>

DECLARE_string(external_loop_closures_file);

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
  int optimize(bool visual_only, bool outlier_rejection);

  int relaxMap();
  int relaxMapMissionsSeparately();

  static constexpr bool kSignalHandlerEnabled = true;

  void relaxMap(vi_map::VIMap* map);
  void addExternalLoopClosureEdgesToMap(
      const data_import_export::LoopClosureEdges& edges, vi_map::VIMap* map);

  // Given two poses and a transformation constraint between them, returns
  // the constraint for two different poses.
  // Subscripts: S source, T target, M map.
  // Calculates T_S2_T2 = T_S2_S * T_S_T * T_T_T2
  pose::Transformation adaptTransformation(
      const pose::Transformation& T_M_S, const pose::Transformation& T_M_T,
      const pose::Transformation& T_M_S2, const pose::Transformation& T_M_T2,
      const pose::Transformation& T_S_T);
};
}  // namespace map_optimization_plugin
#endif  // MAP_OPTIMIZATION_PLUGIN_OPTIMIZER_PLUGIN_H_
