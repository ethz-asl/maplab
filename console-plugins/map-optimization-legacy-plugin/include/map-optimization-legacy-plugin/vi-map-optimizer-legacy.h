#ifndef MAP_OPTIMIZATION_LEGACY_PLUGIN_VI_MAP_OPTIMIZER_LEGACY_H_
#define MAP_OPTIMIZATION_LEGACY_PLUGIN_VI_MAP_OPTIMIZER_LEGACY_H_

#include <string>

#include <ceres/ceres.h>
#include <console-common/command-registerer.h>
#include <map-optimization-legacy/ba-optimization-options.h>
#include <vi-map/unique-id.h>

namespace visualization {
class ViwlsGraphRvizPlotter;
}  // namespace visualization

namespace vi_map {
class VIMap;
}  // namespace vi_map

namespace map_optimization_legacy {
struct OptimizationOptions;
}  // namespace map_optimization_legacy

namespace map_optimization_legacy_plugin {
class VIMapOptimizer {
 public:
  VIMapOptimizer();
  explicit VIMapOptimizer(visualization::ViwlsGraphRvizPlotter* plotter);
  VIMapOptimizer(
      visualization::ViwlsGraphRvizPlotter* plotter,
      bool signal_handler_enabled);

  enum ConsistencyStatus {
    kInconsistent = common::kCustomStatusOffset,
    kNoData,
    kUpdateRejected,
    kNoOverlapFound
  };

  int optimizeUsingDefaultOptions(vi_map::VIMap* map);
  int optimize(
      const map_optimization_legacy::BaOptimizationOptions& options,
      vi_map::VIMap* map);
  int optimize(
      const map_optimization_legacy::BaOptimizationOptions& options,
      vi_map::VIMap* map, ceres::Solver::Summary* summary);

  int relaxPosegraphBasedOnLoopclosureEdges(vi_map::VIMap* map);

  void visualizePosegraph(const vi_map::VIMap& map) const;

 private:
  visualization::ViwlsGraphRvizPlotter* plotter_;
  bool signal_handler_enabled_;
};

}  // namespace map_optimization_legacy_plugin

#endif  // MAP_OPTIMIZATION_LEGACY_PLUGIN_VI_MAP_OPTIMIZER_LEGACY_H_
