#ifndef MAP_OPTIMIZATION_VI_MAP_OPTIMIZER_H_
#define MAP_OPTIMIZATION_VI_MAP_OPTIMIZER_H_

#include <string>
#include <vector>

#include <ceres/ceres.h>
#include <map-optimization/outlier-rejection-solver.h>
#include <map-optimization/vi-optimization-builder.h>
#include <vi-map/unique-id.h>

namespace visualization {
class ViwlsGraphRvizPlotter;
}  // namespace visualization

namespace vi_map {
class VIMap;
}  // namespace vi_map

namespace map_optimization {

class VIMapOptimizer {
 public:
  VIMapOptimizer(
      const visualization::ViwlsGraphRvizPlotter* plotter,
      bool signal_handler_enabled);

  bool optimize(
      const map_optimization::ViProblemOptions& options,
      const vi_map::MissionIdSet& missions_to_optimize, vi_map::VIMap* map,
      OptimizationProblemResult* result);

  bool optimize(
      const map_optimization::ViProblemOptions& options,
      const vi_map::MissionIdSet& missions_to_optimize, vi_map::VIMap* map);
  bool optimizeWithCov(
      const map_optimization::ViProblemOptions& options,
      const vi_map::MissionIdSet& missions_to_optimize,
      const pose_graph::VertexIdList& vertices, vi_map::VIMap* map,
      OptimizationProblemResult* result);

  std::vector<double> getResidualsForVertices(
      const map_optimization::ViProblemOptions& options,
      const vi_map::MissionIdSet& missions_to_optimize,
      const pose_graph::VertexIdList& vertices, vi_map::VIMap* map);

 private:
  const visualization::ViwlsGraphRvizPlotter* plotter_;
  bool signal_handler_enabled_;
};

}  // namespace map_optimization

#endif  // MAP_OPTIMIZATION_VI_MAP_OPTIMIZER_H_
