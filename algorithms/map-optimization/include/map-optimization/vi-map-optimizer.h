#ifndef MAP_OPTIMIZATION_VI_MAP_OPTIMIZER_H_
#define MAP_OPTIMIZATION_VI_MAP_OPTIMIZER_H_

#include <string>

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
      visualization::ViwlsGraphRvizPlotter* plotter,
      bool signal_handler_enabled);

  bool optimizeVisualInertial(
      const map_optimization::ViProblemOptions& options,
      const vi_map::MissionIdSet& missions_to_optimize,
      const map_optimization::OutlierRejectionSolverOptions* const
          outlier_rejection_options,
      vi_map::VIMap* map);

  bool optimizeVisualInertial(
      const map_optimization::ViProblemOptions& options,
      const ceres::Solver::Options& solver_options,
      const vi_map::MissionIdSet& missions_to_optimize,
      const map_optimization::OutlierRejectionSolverOptions* const
          outlier_rejection_options,
      vi_map::VIMap* map);

 private:
  visualization::ViwlsGraphRvizPlotter* plotter_;
  bool signal_handler_enabled_;
};

}  // namespace map_optimization

#endif  // MAP_OPTIMIZATION_VI_MAP_OPTIMIZER_H_
